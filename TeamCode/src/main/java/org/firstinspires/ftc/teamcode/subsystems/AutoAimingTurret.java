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
    private static final double DEGREES = 360;
    private static final double ENCODERTICKS = 537.7;
    private static final double TICKS_PER_DEGREE =  ENCODERTICKS / DEGREES;
    private static final double MAX_POWER = 0.8;
    private static final double RECENTER_POWER = 0.4;
    private static final double TARGET_LOST_TIMEOUT = 2.0;
    private static final double HOME_ANGLE = 0.0;

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

    private boolean leftPressed() { return !limitLeft.getState(); }
    private boolean rightPressed() { return !limitRight.getState(); }

    private boolean hasValidTarget(LLResult r) {
        if (r == null || !r.isValid()) return false;
        List<LLResultTypes.FiducialResult> list = r.getFiducialResults();
        return list != null && list.size() > 0;
    }

    public void runTurret() {
        robotHeading = Math.toDegrees(follower.getHeading());
        turretAngle = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;

        LLResult result = limelight.getLatestResult();
        boolean valid = hasValidTarget(result);

        if (valid) {
            double tx = result.getFiducialResults().get(0).getTargetXDegrees();
            lastKnownFieldAngle = robotHeading + turretAngle + tx;
            targetWasVisible = true;
            targetLostTimer.reset();
        }

        if (targetWasVisible && targetLostTimer.seconds() < TARGET_LOST_TIMEOUT) {
            turretStatus = "Tracking";

            desiredTurretAngle = lastKnownFieldAngle - robotHeading;
            int targetTicks = (int) (desiredTurretAngle * TICKS_PER_DEGREE);

            if ((targetTicks < turretMotor.getCurrentPosition() && leftPressed()) ||
                    (targetTicks > turretMotor.getCurrentPosition() && rightPressed())) {
                turretMotor.setPower(0);
                return;
            }

            turretMotor.setTargetPosition(targetTicks);
            turretMotor.setPower(MAX_POWER);
        } else {
            turretStatus = "Searching";

            double error = HOME_ANGLE - turretAngle;
            int homeTicks = (int) (HOME_ANGLE * TICKS_PER_DEGREE);

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
    }

    // --- Manual Controls ---
    public void manualUpdate(Gamepad gp) {
        if (gp.square && !previousSquare) {
            shooterBlocked = !shooterBlocked;
            shooterBlocker.setPosition(shooterBlocked ?
                    Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION :
                    Constants.TurretConfig.SHOOTER_BLOCKER_ZERO_POSITION);
        }
        previousSquare = gp.square;

        if (gp.start && !previousPipelineToggle) {
            currentPipeline = (currentPipeline == 0 ? 1 : 0);
            limelight.pipelineSwitch(currentPipeline);
            gp.rumble(100);
        }
        previousPipelineToggle = gp.start;
    }

    public void manualNudgeButtons(Gamepad gamepad, double nudgePower) {
        boolean leftNudge = gamepad.left_bumper;  // L1
        boolean rightNudge = gamepad.right_bumper; // R1

        if (leftNudge || rightNudge) {
            turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            double power = 0;
            if (leftNudge && !leftPressed()) power = -nudgePower;
            if (rightNudge && !rightPressed()) power = nudgePower;

            turretMotor.setPower(power);
        } else {
            turretMotor.setPower(0);
            turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
    }

    public void stop() {
        try {
            if (turretMotor != null) turretMotor.setPower(0);
            if (limelight != null) limelight.stop();
        } catch (Exception ignored) { }
    }

    // --- Getters for telemetry ---
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
}
