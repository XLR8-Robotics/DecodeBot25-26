package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;

import org.firstinspires.ftc.teamcode.config.Constants;

public class AutoAimingTurret {

    // --- Hardware ---
    private DcMotorEx turretMotor;
    private Limelight3A limelight;
    private Follower follower;
    private final DigitalChannel turretLimitLeft;
    private final DigitalChannel turretLimitRight;
    private final Servo shooterBlocker;

    // --- Constants ---
    private static final double TICKS_PER_DEGREE = 537.7 / 360.0; // adjust for gear ratio if needed
    private static final double P = 10.0;
    private static final double I = 0.0;
    private static final double D = 0.0;
    private static final double F = 0.0;
    private static final double MAX_POWER = 0.8;
    private static final double TARGET_LOST_TIMEOUT = 2.0; // seconds

    private static final double TURRET_HOME_ANGLE = 0.0; // degrees
    private static final double RECENTER_POWER = 0.4;

    // --- State Variables ---
    private double lastKnownTargetAngleField = 0.0;
    private boolean targetWasVisible = false;
    private final ElapsedTime targetLostTimer = new ElapsedTime();
    private boolean isShooterBlocked = true;
    private boolean previousSquareButtonState = false;

    private double robotHeading;
    private double currentTurretAngle;
    private String hasValidTarget;
    private double desiredTurretAngle;
    private String turretStatus;
    private int currentPipeline;
    private boolean previousCircleButtonState;


    public enum TurrentStatus {
        INITIALIZED("Initialized"),
        TRACKING("Tracking"),
        SEARCHING("Searching");
        private final String description;
        TurrentStatus(String description) { this.description = description; }
        public String getDescription() { return description; }
    }

    public AutoAimingTurret(HardwareMap hardwareMap, Follower follower) {
        try {
            this.follower = follower;

            // --- Turret Motor Initialization ---
            turretMotor = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.TURRET_MOTOR);
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
            turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);

            // --- Shooter Blocker and Limits ---
            shooterBlocker = hardwareMap.get(Servo.class, Constants.HardwareConfig.SHOOTER_BLOCKER);
            turretLimitLeft = hardwareMap.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_LEFT);
            turretLimitRight = hardwareMap.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_RIGHT);

            turretLimitLeft.setMode(DigitalChannel.Mode.INPUT);
            turretLimitRight.setMode(DigitalChannel.Mode.INPUT);

            // --- Limelight Initialization ---
            limelight = hardwareMap.get(Limelight3A.class, Constants.HardwareConfig.LIMELIGHT_NAME);
            limelight.pipelineSwitch(0);
            limelight.start();

            targetLostTimer.reset();
            turretStatus = TurrentStatus.INITIALIZED.getDescription();

            setPipeline(0);

        } catch (Exception e) {
            throw new RuntimeException("Turret initialization failed: " + e.getMessage());
        }
    }

    public void RunTurret() {
        try {
            robotHeading = getRobotHeading();
            currentTurretAngle = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;

            LLResult result = limelight.getLatestResult();
            hasValidTarget = hasValidTarget(result) ? "Yes" : "No";

            if (hasValidTarget(result)) {
                double tx = result.getFiducialResults().get(0).getTargetXDegrees();
                lastKnownTargetAngleField = robotHeading + currentTurretAngle + tx;
                targetWasVisible = true;
                targetLostTimer.reset();
            }

            if (targetWasVisible && targetLostTimer.seconds() < TARGET_LOST_TIMEOUT) {
                desiredTurretAngle = lastKnownTargetAngleField - robotHeading;
                int targetPositionTicks = (int) (desiredTurretAngle * TICKS_PER_DEGREE);

                // --- Limit Switch Protection ---
                if ((targetPositionTicks < turretMotor.getCurrentPosition() && isLeftLimitPressed()) ||
                        (targetPositionTicks > turretMotor.getCurrentPosition() && isRightLimitPressed())) {
                    turretMotor.setPower(0);
                } else {
                    turretMotor.setTargetPosition(targetPositionTicks);
                    turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    turretMotor.setPower(MAX_POWER);
                }

                turretStatus = TurrentStatus.TRACKING.getDescription();

            } else {
                // --- Smooth Re-Centering ---
                targetWasVisible = false;
                turretStatus = TurrentStatus.SEARCHING.getDescription();
                double error = TURRET_HOME_ANGLE - currentTurretAngle;
                int targetTicks = (int) (TURRET_HOME_ANGLE * TICKS_PER_DEGREE);

                if (Math.abs(error) > 1.0) {
                    // Respect limit switches while re-centering
                    if ((error < 0 && !isLeftLimitPressed()) || (error > 0 && !isRightLimitPressed())) {
                        turretMotor.setTargetPosition(targetTicks);
                        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        turretMotor.setPower(RECENTER_POWER);
                    } else {
                        turretMotor.setPower(0);
                    }
                } else {
                    turretMotor.setPower(0);
                }
            }
        } catch (Exception e) {
            turretMotor.setPower(0);
        }
    }
    public double getRobotHeading() { return Math.toDegrees(follower.getHeading()); }
    public double getCurrentRobotHeading() { return robotHeading; }
    public double getCurrentTurretAngle() { return currentTurretAngle; }
    public double getDesiredTurretAngle() { return desiredTurretAngle; }
    public double getLastKnownTargetAngleField() { return lastKnownTargetAngleField; }
    public double getTargetLostTimer() { return targetLostTimer.seconds(); }
    public String getHasValidTarget() { return hasValidTarget; }
    public String getTurretStatus() { return turretStatus; }
    private boolean hasValidTarget(LLResult result) {
        if (result == null || !result.isValid()) return false;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        return fiducials != null && !fiducials.isEmpty();
    }
    public boolean isLeftLimitPressed() { return !turretLimitLeft.getState(); }
    public boolean isRightLimitPressed() { return !turretLimitRight.getState(); }
    public void manualUpdate(Gamepad gamepad) {
        boolean currentSquareButtonState = gamepad.square;
        if (currentSquareButtonState && !previousSquareButtonState) {
            isShooterBlocked = !isShooterBlocked;
            if (isShooterBlocked) setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION);
            else setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_ZERO_POSITION);
        }

        previousSquareButtonState = currentSquareButtonState;

        boolean currentCircleButtonState = gamepad.start;
        if (currentCircleButtonState && !previousCircleButtonState) {
            currentPipeline = (currentPipeline == 0) ? 1 : 0;
            setPipeline(currentPipeline);
            gamepad.rumble(1);
        }
        previousCircleButtonState = currentCircleButtonState;
    }
    public void setShooterBlocked() { setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION); }
    public void setShooterUnBlocked() { setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_ZERO_POSITION); }
    public void setShooterBlockerPosition(double position) { shooterBlocker.setPosition(position); }
    public void setPipeline(int pipelineIndex) {
        try {
            if (limelight != null) {
                limelight.pipelineSwitch(pipelineIndex);
            }
        } catch (Exception e) {
            // Optional: log the error or ignore
            System.out.println("Failed to switch Limelight pipeline: " + e.getMessage());
        }
    }
    public void stop() {
        try {
            if (turretMotor != null) turretMotor.setPower(0);
            if (limelight != null) limelight.stop();
        } catch (Exception e) { }
    }
}
