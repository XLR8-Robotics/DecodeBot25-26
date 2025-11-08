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

    public Robot() {}

    public void init(HardwareMap hardwareMap) {
        drivetrain = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap);
    }

    /**
     * This is the main update loop for the robot.
     * It handles both manual control and the automated launch sequence.
     */
    public void update(Gamepad gamepad) {
        // Update the limelight data first
        limelight.update();

        // --- Automated Turret Aiming ---
        handleAutomatedAiming();

        // --- Launch Sequence Logic ---
        // The launch sequence takes priority over all manual controls.
        handleLaunchSequence(gamepad);

        // If the launch sequence is not active, allow manual control.
        if (currentLaunchState == LaunchSequenceState.IDLE) {
            // Drivetrain Control
            double forward = -gamepad.left_stick_y;
            double strafe = gamepad.left_stick_x;
            double turn = gamepad.right_stick_x;
            drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, -strafe), turn));

            // Subsystem Control (excluding turret)
            intake.update(gamepad);
            shooter.update(gamepad);
        }
    }

    private void handleAutomatedAiming() {
        double turretPower = 0.0;
        boolean hasValidTarget = false;
        double targetTx = 0.0;

        // Get the target tag ID based on selected side
        int targetTagId = (currentTargetSide == TargetSide.RED) 
            ? PatternIdentifier.TOWER_RED 
            : PatternIdentifier.TOWER_BLUE;

        if (limelight.hasTarget()) {
            // Check all detected fiducials to find our target tag
            // This prioritizes the selected target even if multiple tags are visible
            List<LLResultTypes.FiducialResult> fiducials = limelight.getFiducialResults();
            
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if (fiducial.getFiducialId() == targetTagId) {
                    hasValidTarget = true;
                    // Use the tx value from the specific target tag
                    targetTx = fiducial.getTargetXDegrees();
                    break;
                }
            }
        }

        if (hasValidTarget) {
            // Reset oscillation timer when target is found
            oscillationTimer.reset();
            // P-controller for aiming using the target tag's tx value
            turretPower = targetTx * Constants.TurretAimingConfig.AIMING_KP;
        } else if (Constants.TurretAimingConfig.ENABLE_OSCILLATION) {
            // No target found - oscillate to search for target
            turretPower = calculateOscillationPower();
        }

        turret.setPower(turretPower);
    }

    /**
     * Calculates the oscillation power for the turret when no target is found.
     * Uses a sine wave pattern to smoothly oscillate the turret left and right.
     * Respects limit switches to prevent hitting physical limits.
     * 
     * @return The power to apply to the turret motor (-1.0 to 1.0)
     */
    private double calculateOscillationPower() {
        // Check limit switches first
        boolean atLeftLimit = turret.isLeftLimitPressed();
        boolean atRightLimit = turret.isRightLimitPressed();

        // If at limits, reverse direction
        if (atLeftLimit) {
            oscillationTimer.reset(); // Reset to start oscillation in opposite direction
            return Constants.TurretAimingConfig.OSCILLATION_SPEED; // Move right
        }
        if (atRightLimit) {
            oscillationTimer.reset(); // Reset to start oscillation in opposite direction
            return -Constants.TurretAimingConfig.OSCILLATION_SPEED; // Move left
        }

        // Use sine wave for smooth oscillation
        // Sine wave ranges from -1 to 1, which we scale by oscillation speed
        double timeSeconds = oscillationTimer.milliseconds() / 1000.0;
        double periodSeconds = Constants.TurretAimingConfig.OSCILLATION_PERIOD_MS / 1000.0;
        double sineValue = Math.sin(2.0 * Math.PI * timeSeconds / periodSeconds);
        
        return sineValue * Constants.TurretAimingConfig.OSCILLATION_SPEED;
    }

    private void handleLaunchSequence(Gamepad gamepad) {
        // --- Cancel Logic ---
        // Detect a triple press of the cross button to cancel the sequence.
        if (gamepad.cross && crossPressTimer.milliseconds() > Constants.LaunchSequenceConfig.TRIPLE_PRESS_TIMEOUT_MS) {
            crossPressCount = 1;
            crossPressTimer.reset();
        } else if (gamepad.cross && crossPressCount > 0) {
            crossPressCount++;
        }

        if (crossPressCount >= 3) {
            if (currentLaunchState != LaunchSequenceState.IDLE) {
                currentLaunchState = LaunchSequenceState.CANCELLED;
                sequenceTimer.reset(); // Reset timer for the cancel action
            }
            crossPressCount = 0; // Reset counter
        }

        // --- State Machine ---
        switch (currentLaunchState) {
            case IDLE:
                // Start the sequence on a single press of the cross button.
                if (gamepad.cross && crossPressCount == 1 && crossPressTimer.milliseconds() < 200) { // check for a single press
                    startLaunchSequence();
                }
                break;

            case SPOOLING:
                // Wait for the shooter to spin up.
                if (sequenceTimer.milliseconds() >= Constants.LaunchSequenceConfig.SHOOTER_SPIN_UP_TIME_MS) {
                    currentLaunchState = LaunchSequenceState.FEEDING;
                }
                break;

            case FEEDING:
                // Run the intake until the last ball is detected.
                intake.setPower(Constants.IntakeConfig.INTAKE_SPEED);
                if (!intake.isObjectDetected()) { // Assuming this means the last ball has passed
                    currentLaunchState = LaunchSequenceState.LIFTING;
                    sequenceTimer.reset();
                }
                break;

            case LIFTING:
                // Lift the servo up, then down.
                intake.setLiftPosition(Constants.IntakeConfig.LIFT_SERVO_LIFTING_POSITION);
                if (sequenceTimer.milliseconds() > 500) { // Wait half a second
                    intake.setLiftPosition(Constants.IntakeConfig.LIFT_SERVO_NOT_LIFTING_POSITION);
                    currentLaunchState = LaunchSequenceState.FINISHING;
                }
                break;

            case FINISHING:
                // Shut down all motors and reset.
                stopAllMotors();
                turret.setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION);
                currentLaunchState = LaunchSequenceState.IDLE;
                break;

            case CANCELLED:
                // Reverse the intake for a short time.
                intake.setPower(-Constants.IntakeConfig.INTAKE_SPEED);
                if (sequenceTimer.milliseconds() >= Constants.LaunchSequenceConfig.INTAKE_REVERSE_TIME_MS) {
                    stopAllMotors();
                    turret.setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION);
                    currentLaunchState = LaunchSequenceState.IDLE;
                }
                break;
        }
    }

    private void startLaunchSequence() {
        currentLaunchState = LaunchSequenceState.SPOOLING;
        sequenceTimer.reset();
        
        double distance = limelight.getDistanceToTarget();

        // Get the interpolated power and hood values from our lookup table
        ShooterTable.ShotParams shot = ShooterTable.getInterpolatedShot(distance);

        // Use the calculated values if a target is visible, otherwise use defaults
        if (limelight.hasTarget()) {
            shooter.setPower(shot.power);
            shooter.setHoodPosition(shot.hood);
        } else {
            shooter.setPower(Constants.ShooterConfig.SHOOTER_SPEED);
            shooter.setHoodPosition(Constants.ShooterConfig.HOOD_CENTER);
        }

        // move the blocker out of the way.
        turret.setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_ZERO_POSITION);
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

    /**
     * Gets the currently selected target side.
     * @return The current target side (RED or BLUE)
     */
    public TargetSide getTargetSide() {
        return currentTargetSide;
    }

    /**
     * Sets the target side for turret aiming.
     * @param side The target side to aim at (RED or BLUE)
     */
    public void setTargetSide(TargetSide side) {
        this.currentTargetSide = side;
    }

    /**
     * Gets the current target tag ID based on the selected side.
     * @return The AprilTag ID for the current target side
     */
    public int getCurrentTargetTagId() {
        return (currentTargetSide == TargetSide.RED) 
            ? PatternIdentifier.TOWER_RED 
            : PatternIdentifier.TOWER_BLUE;
    }
}
