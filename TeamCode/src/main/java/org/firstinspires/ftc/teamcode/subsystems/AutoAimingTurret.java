package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.config.Constants;

import java.util.List;

public class AutoAimingTurret {

    // --- Hardware ---
    private DcMotorEx turretMotor;
    private Limelight3A limelight;
    private Follower follower;

    private final DigitalChannel limitLeft;
    private final DigitalChannel limitRight;
    private final Servo shooterBlocker;

    // --- Constants ---
    private static final double TICKS_PER_DEGREE = 5.87;
    private static final double MAX_POWER = 0.8;
    private static final double RECENTER_POWER = 0.4;
    private static final double TARGET_LOST_TIMEOUT = 2.0;
    private static final double HOME_ANGLE = 0.0;

    // --- Dynamic zero calibration ---
    private double zeroOffset = 0.0;
    private boolean zeroCalibrated = false;

    // --- Safe turret sweep (0–360° system) ---
    private static final double MAX_RIGHT = 55.0;  // clockwise from back
    private static final double MAX_LEFT  = 305.0; // counter-clockwise from back

    // --- State ---
    private boolean targetWasVisible = false;
    private double lastKnownFieldAngle = 0;
    private double robotHeading = 0;
    private double turretAngle = 0;
    private double desiredTurretAngle = 0;

    private final ElapsedTime targetLostTimer = new ElapsedTime();

    private boolean previousSquare = false;
    private boolean shooterBlocked = true;

    private boolean previousPipelineToggle = false;
    private int currentPipeline = 0;

    private String turretStatus = "Initialized";

    public AutoAimingTurret(HardwareMap map, Follower follower) {
        this.follower = follower;

        // Motor
        turretMotor = map.get(DcMotorEx.class, Constants.HardwareConfig.TURRET_MOTOR);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Servo
        shooterBlocker = map.get(Servo.class, Constants.HardwareConfig.SHOOTER_BLOCKER);

        // Limit switches
        limitLeft = map.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_LEFT);
        limitRight = map.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_RIGHT);
        limitLeft.setMode(DigitalChannel.Mode.INPUT);
        limitRight.setMode(DigitalChannel.Mode.INPUT);

        // Limelight
        limelight = map.get(Limelight3A.class, Constants.HardwareConfig.LIMELIGHT_NAME);
        limelight.start();
        limelight.pipelineSwitch(0);

        targetLostTimer.reset();
    }

    // --- Limit switch helpers ---
    private boolean leftPressed() { return !limitLeft.getState(); }
    private boolean rightPressed() { return !limitRight.getState(); }

    private boolean hasValidTarget(LLResult r) {
        if (r == null || !r.isValid()) return false;
        List<LLResultTypes.FiducialResult> list = r.getFiducialResults();
        return list != null && list.size() > 0;
    }

    // ===================================================================
    // AUTO AIM MAIN LOOP
    // ===================================================================
    public void runTurret() {

        robotHeading = Math.toDegrees(follower.getHeading());
        turretAngle = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;

        LLResult result = limelight.getLatestResult();
        boolean valid = hasValidTarget(result);

        // -------------------------------
        // 🟦 One-time ZERO calibration
        // -------------------------------
        if (valid && !zeroCalibrated) {
            double tx = result.getFiducialResults().get(0).getTargetXDegrees();
            if (Math.abs(tx) < 3.0) { // calibrate only if near center
                zeroOffset = robotHeading + turretAngle + tx;
                zeroCalibrated = true;
                turretStatus = "Zero Calibrated";
            }
        }

        // -------------------------------
        // 🔵 Target visible
        // -------------------------------
        if (valid) {
            double tx = result.getFiducialResults().get(0).getTargetXDegrees();
            lastKnownFieldAngle = robotHeading + turretAngle + zeroOffset + tx;
            targetWasVisible = true;
            targetLostTimer.reset();
        }

        // -------------------------------
        // 🟢 Tracking mode
        // -------------------------------
        if (targetWasVisible && targetLostTimer.seconds() < TARGET_LOST_TIMEOUT) {
            turretStatus = "Tracking";

            desiredTurretAngle = lastKnownFieldAngle - robotHeading - zeroOffset;

            // Convert to 0–360 range
            double desired360 = (desiredTurretAngle + 360) % 360;

            // Clamp into triangle-safe arc
            if (desired360 > MAX_RIGHT && desired360 < MAX_LEFT) {
                desired360 = (desired360 > 180) ? MAX_LEFT : MAX_RIGHT;
            }

            int targetTicks = (int)(desired360 * TICKS_PER_DEGREE);

            // Limit switches hard stop
            if ((targetTicks < turretMotor.getCurrentPosition() && leftPressed()) ||
                    (targetTicks > turretMotor.getCurrentPosition() && rightPressed())) {
                turretMotor.setPower(0);
            } else {
                turretMotor.setTargetPosition(targetTicks);
                turretMotor.setPower(MAX_POWER);
            }
            return;
        }

        // -------------------------------
        // 🔴 No target → recenter
        // -------------------------------
        turretStatus = "Searching";

        double error = HOME_ANGLE - turretAngle;
        int homeTicks = (int)(HOME_ANGLE * TICKS_PER_DEGREE);

        if (Math.abs(error) > 1.0) {
            if ((error < 0 && !leftPressed()) || (error > 0 && !rightPressed())) {
                turretMotor.setTargetPosition(homeTicks);
                turretMotor.setPower(RECENTER_POWER);
            } else {
                turretMotor.setPower(0);
            }
        } else {
            turretMotor.setPower(0);
        }
    }

    // ===================================================================
    // Manual controls
    // ===================================================================
    public void manualUpdate(Gamepad gp) {

        if (gp.square && !previousSquare) {
            shooterBlocked = !shooterBlocked;
            shooterBlocker.setPosition(
                    shooterBlocked ?
                            Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION :
                            Constants.TurretConfig.SHOOTER_BLOCKER_ZERO_POSITION
            );
        }
        previousSquare = gp.square;

        // Pipeline toggle
        if (gp.start && !previousPipelineToggle) {
            currentPipeline = (currentPipeline == 0 ? 1 : 0);
            limelight.pipelineSwitch(currentPipeline);
            gp.rumble(100);
        }
        previousPipelineToggle = gp.start;
    }

    public void stop() {
        try {
            if (turretMotor != null) turretMotor.setPower(0);
            if (limelight != null) limelight.stop();
        } catch (Exception ignored) { }
    }

    // ===================================================================
    // TELEMETRY GETTERS (unchanged)
    // ===================================================================
    public double getCurrentRobotHeading() { return robotHeading; }
    public double getCurrentTurretAngle() { return turretAngle; }
    public double getDesiredTurretAngle() { return desiredTurretAngle; }
    public double getLastKnownTargetAngleField() { return lastKnownFieldAngle; }
    public double getTargetLostTimer() { return targetLostTimer.seconds(); }
    public String getHasValidTarget() { return hasValidTarget(limelight.getLatestResult()) ? "Yes" : "No"; }
    public String getTurretStatus() { return turretStatus; }
    public boolean isLeftLimitPressed() { return leftPressed(); }
    public boolean isRightLimitPressed() { return rightPressed(); }
    public boolean isShooterBlocked() { return shooterBlocked; }
    public double getShooterBlockerPosition() { return shooterBlocker.getPosition(); }
}
