package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.teamcode.utils.PatternIdentifier;
import org.firstinspires.ftc.teamcode.utils.ShooterTable;

import java.util.List;

public class Robot {

    // Subsystems
    public MecanumDrive drivetrain;
    public Intake intake;
    public Shooter shooter;
    public Turret turret;
    public Limelight limelight;

    // Enum for target side selection
    public enum TargetSide {
        RED,
        BLUE
    }

    // State Machine for the Launch Sequence
    private enum LaunchSequenceState {
        IDLE,           // Not running, waiting for start command
        SPOOLING,       // Shooter motor is spinning up
        FEEDING,        // Intake is running to feed balls
        LIFTING,        // Lift servo is moving up and down
        FINISHING,      // Sequence is complete, shutting down motors
        CANCELLED,      // Sequence was cancelled, reversing intake
    }
    private LaunchSequenceState currentLaunchState = LaunchSequenceState.IDLE;

    // Timers and press counters for the sequence
    private final ElapsedTime sequenceTimer = new ElapsedTime();
    private int crossPressCount = 0;
    private final ElapsedTime crossPressTimer = new ElapsedTime();

    // Timer for turret oscillation when no target is found
    private final ElapsedTime oscillationTimer = new ElapsedTime();

    // Target side selection (default to BLUE)
    private TargetSide currentTargetSide = TargetSide.BLUE;

    // Auto-aim state
    private boolean autoAimEnabled = false;
    private boolean previousTriangleButtonState = false;

    // PID controller variables
    private double integral = 0;
    private double previousError = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    public Robot() {}

    public void init(HardwareMap hardwareMap) {
        drivetrain = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap);
    }

    public void update(Gamepad gamepad) {
        limelight.update();

        // --- Auto-Aim and Manual Turret Control ---
        handleTurretControl(gamepad);

        // --- Launch Sequence Logic ---
        handleLaunchSequenceInput(gamepad);
        updateLaunchSequence();

        // If the launch sequence is not active, allow manual control.
        if (currentLaunchState == LaunchSequenceState.IDLE) {
            // Drivetrain Control
            double forward = -gamepad.left_stick_y * Constants.DrivetrainConfig.DRIVE_SPEED_MULTIPLIER;
            double strafe = gamepad.left_stick_x * Constants.DrivetrainConfig.DRIVE_SPEED_MULTIPLIER;
            double turn = gamepad.right_stick_x * Constants.DrivetrainConfig.DRIVE_SPEED_MULTIPLIER;
            drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, -strafe), turn));

            // Subsystem Control (excluding turret, which is handled in handleTurretControl)
            intake.update(gamepad);
            shooter.update(gamepad);
        }
    }

    public void manualUpdate(Gamepad gamepad) {
        // Drivetrain Control
        double forward = -gamepad.left_stick_y * Constants.DrivetrainConfig.DRIVE_SPEED_MULTIPLIER;
        double strafe = gamepad.left_stick_x * Constants.DrivetrainConfig.DRIVE_SPEED_MULTIPLIER;
        double turn = gamepad.right_stick_x * Constants.DrivetrainConfig.DRIVE_SPEED_MULTIPLIER;
        drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, -strafe), turn));

        // Subsystem Control
        intake.update(gamepad);
        shooter.update(gamepad);
        turret.update(gamepad);
    }

    private void handleTurretControl(Gamepad gamepad) {
        // Toggle auto-aim with the triangle button
        boolean currentTriangleButtonState = gamepad.triangle;
        if (currentTriangleButtonState && !previousTriangleButtonState) {
            autoAimEnabled = !autoAimEnabled;
            // Reset PID when toggling auto-aim
            if (autoAimEnabled) {
                integral = 0;
                previousError = 0;
                pidTimer.reset();
            }
        }
        previousTriangleButtonState = currentTriangleButtonState;

        if (autoAimEnabled) {
            handleAutomatedAiming();
        } else {
            // When auto-aim is off, the turret is manually controlled.
            // The turret's update method handles manual bumper controls and the shooter blocker.
            turret.update(gamepad);
        }
    }

    private void handleAutomatedAiming() {
        double turretPower = 0.0;
        boolean hasValidTarget = false;
        double targetTx = 0.0;

        int targetTagId = (currentTargetSide == TargetSide.RED)
                ? PatternIdentifier.TOWER_RED
                : PatternIdentifier.TOWER_BLUE;

        if (limelight.hasTarget()) {
            List<LLResultTypes.FiducialResult> fiducials = limelight.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if (fiducial.getFiducialId() == targetTagId) {
                    hasValidTarget = true;
                    targetTx = fiducial.getTargetXDegrees();
                    break;
                }
            }
        }

        if (hasValidTarget) {
            oscillationTimer.reset();

            // PID controller logic
            double error = targetTx;
            double dt = pidTimer.seconds();
            pidTimer.reset();

            // Integral term
            integral += error * dt;

            // Derivative term
            double derivative = (error - previousError) / dt;

            // PID formula
            turretPower = (Constants.TurretAimingConfig.AIMING_KP * error) +
                          (Constants.TurretAimingConfig.AIMING_KI * integral) +
                          (Constants.TurretAimingConfig.AIMING_KD * derivative);

            previousError = error;
        } else if (Constants.TurretAimingConfig.ENABLE_OSCILLATION) {
            turretPower = calculateOscillationPower();
        } else {
            // Reset PID if no target is found and oscillation is off
            integral = 0;
            previousError = 0;
        }

        turret.setPower(turretPower);
    }

    private double calculateOscillationPower() {
        boolean atLeftLimit = turret.isLeftLimitPressed();
        boolean atRightLimit = turret.isRightLimitPressed();

        if (atLeftLimit) {
            oscillationTimer.reset();
            return Constants.TurretAimingConfig.OSCILLATION_SPEED;
        }
        if (atRightLimit) {
            oscillationTimer.reset();
            return -Constants.TurretAimingConfig.OSCILLATION_SPEED;
        }

        double timeSeconds = oscillationTimer.milliseconds() / 1000.0;
        double periodSeconds = Constants.TurretAimingConfig.OSCILLATION_PERIOD_MS / 1000.0;
        double sineValue = Math.sin(2.0 * Math.PI * timeSeconds / periodSeconds);

        return sineValue * Constants.TurretAimingConfig.OSCILLATION_SPEED;
    }

    private void handleLaunchSequenceInput(Gamepad gamepad) {
        if (gamepad.cross && crossPressTimer.milliseconds() > Constants.LaunchSequenceConfig.TRIPLE_PRESS_TIMEOUT_MS) {
            crossPressCount = 1;
            crossPressTimer.reset();
        } else if (gamepad.cross && crossPressCount > 0) {
            crossPressCount++;
        }

        if (crossPressCount >= 3) {
            cancelLaunchSequence();
            crossPressCount = 0;
        }

        if (currentLaunchState == LaunchSequenceState.IDLE) {
            if (gamepad.cross && crossPressCount == 1 && crossPressTimer.milliseconds() < 200) {
                startLaunchSequence();
            }
        }
    }

    public void updateLaunchSequence() {
        switch (currentLaunchState) {
            case IDLE:
                break;
            case SPOOLING:
                if (sequenceTimer.milliseconds() >= Constants.LaunchSequenceConfig.SHOOTER_SPIN_UP_TIME_MS) {
                    currentLaunchState = LaunchSequenceState.FEEDING;
                }
                break;
            case FEEDING:
                intake.setPower(Constants.IntakeConfig.INTAKE_SPEED);
                if (!intake.isObjectDetected()) {
                    currentLaunchState = LaunchSequenceState.LIFTING;
                    sequenceTimer.reset();
                }
                break;
            case LIFTING:
                intake.setLiftPosition(Constants.IntakeConfig.LIFT_SERVO_LIFTING_POSITION);
                if (sequenceTimer.milliseconds() > 500) {
                    intake.setLiftPosition(Constants.IntakeConfig.LIFT_SERVO_NOT_LIFTING_POSITION);
                    currentLaunchState = LaunchSequenceState.FINISHING;
                }
                break;
            case FINISHING:
                stopAllMotors();
                turret.setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION);
                currentLaunchState = LaunchSequenceState.IDLE;
                break;
            case CANCELLED:
                intake.setPower(-Constants.IntakeConfig.INTAKE_SPEED);
                if (sequenceTimer.milliseconds() >= Constants.LaunchSequenceConfig.INTAKE_REVERSE_TIME_MS) {
                    stopAllMotors();
                    turret.setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION);
                    currentLaunchState = LaunchSequenceState.IDLE;
                }
                break;
        }
    }

    public void startLaunchSequence() {
        if (currentLaunchState != LaunchSequenceState.IDLE) return;
        currentLaunchState = LaunchSequenceState.SPOOLING;
        sequenceTimer.reset();

        double distance = limelight.getDistanceToTarget();
        ShooterTable.ShotParams shot = ShooterTable.getInterpolatedShot(distance);

        if (limelight.hasTarget()) {
            shooter.setPower(shot.power);
            shooter.setHoodPosition(shot.hood);
        } else {
            shooter.setPower(Constants.ShooterConfig.SHOOTER_SPEED);
            shooter.setHoodPosition(Constants.ShooterConfig.HOOD_CENTER);
        }

        turret.setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_ZERO_POSITION);
    }

    public void cancelLaunchSequence() {
        if (currentLaunchState != LaunchSequenceState.IDLE) {
            currentLaunchState = LaunchSequenceState.CANCELLED;
            sequenceTimer.reset();
        }
    }

    public void stopAllMotors() {
        drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        intake.setPower(0);
        shooter.setPower(0);
        turret.setPower(0);
    }

    public String getLaunchSequenceState(){
        return currentLaunchState.toString();
    }

    public TargetSide getTargetSide() {
        return currentTargetSide;
    }

    public void setTargetSide(TargetSide side) {
        this.currentTargetSide = side;
    }

    public int getCurrentTargetTagId() {
        return (currentTargetSide == TargetSide.RED)
                ? PatternIdentifier.TOWER_RED
                : PatternIdentifier.TOWER_BLUE;
    }
}
