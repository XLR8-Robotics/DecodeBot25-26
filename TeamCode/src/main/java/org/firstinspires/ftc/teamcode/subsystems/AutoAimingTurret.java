package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.Constants;

public class AutoAimingTurret {

    // -------------------- Hardware --------------------
    private DcMotorEx turretMotor;
    private Servo shooterBlocker;
    private final DigitalChannel limitLeft;
    private final DigitalChannel limitRight;
    private  Limelight3A camera;
    private  Follower follower;

    // -------------------- PID Control --------------------
    private final PIDFController turretPID;
    private static final double TICKS_PER_DEGREE = 5.87;
    private static final double MAX_POWER = 0.8;
    private double offsetDegrees = 0.0;

    // -------------------- Manual State --------------------
    private boolean isShooterBlocked = true;
    private boolean previousSquare = false;

    // -------------------- Vision / Odometry --------------------
    private final ElapsedTime visionTimer = new ElapsedTime();
    private double lastKnownTargetX;
    private double lastKnownTargetY;
    private static final double ODOMETRY_CORRECTION_THRESHOLD = 0.05; // meters or units

    // -------------------- Limits --------------------
    private static final double MAX_RIGHT = 55.0;
    private static final double MAX_LEFT = 305.0;

    // -------------------- Constructor --------------------
    public AutoAimingTurret(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;

        // Use centralized Constants for hardware names
        this.turretMotor = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.TURRET_MOTOR);
        this.shooterBlocker = hardwareMap.get(Servo.class, Constants.HardwareConfig.SHOOTER_BLOCKER);
        this.limitLeft = hardwareMap.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_LEFT);
        this.limitRight = hardwareMap.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_RIGHT);
        this.camera = hardwareMap.get(Limelight3A.class, Constants.HardwareConfig.LIMELIGHT_NAME);

        limitLeft.setMode(DigitalChannel.Mode.INPUT);
        limitRight.setMode(DigitalChannel.Mode.INPUT);

        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        turretPID = new PIDFController(new PIDFCoefficients(0.01, 0.0, 0.0005, 0.0));

        shooterBlocker.setPosition(Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION);
    }


    // ===================================================================
    // Manual control
    // ===================================================================
    public void manualUpdate(Gamepad gamepad) {
        double power = 0;
        if (gamepad.left_bumper && !isLeftLimitPressed()) power = Constants.TurretConfig.TURRET_SPEED;
        else if (gamepad.right_bumper && !isRightLimitPressed()) power = -Constants.TurretConfig.TURRET_SPEED;

        setPower(power);

        if (gamepad.square && !previousSquare) {
            isShooterBlocked = !isShooterBlocked;
            shooterBlocker.setPosition(
                    isShooterBlocked ? Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION
                            : Constants.TurretConfig.SHOOTER_BLOCKER_ZERO_POSITION
            );
        }
        previousSquare = gamepad.square;
    }

    // ===================================================================
    // Set target in field coordinates
    // ===================================================================
    public void setTarget(double targetX, double targetY) {
        lastKnownTargetX = targetX;
        lastKnownTargetY = targetY;
    }

    // ===================================================================
    // Update loop
    // ===================================================================
    public void update() {
        // ---------------- Current robot state ----------------
        Pose currentPose = follower.getPose();
        double currentHeading = Math.toDegrees(follower.getHeading());

        // ---------------- Vision-based odometry correction ----------------
        Pose correctedPose = computeCorrectedOdometry(currentPose, currentHeading);
        if (correctedPose != null) {
            follower.setPose(correctedPose);
            currentPose = correctedPose;
        }

        // ---------------- PID aiming ----------------
        // Compute angle to target
        double angleToTarget = Math.toDegrees(Math.atan2(
                lastKnownTargetY - currentPose.getY(),
                lastKnownTargetX - currentPose.getX()
        ));

        double turretTargetAngle = 180.0 - angleToTarget;

        turretTargetAngle = (turretTargetAngle + 360) % 360;

        boolean targetInRange = turretTargetAngle >= MAX_RIGHT && turretTargetAngle <= MAX_LEFT;

        if (targetInRange) {
            turretPID.setTargetPosition(turretTargetAngle + offsetDegrees);

            double currentAngle = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
            turretPID.updatePosition(currentAngle);
            double power = Math.max(-MAX_POWER, Math.min(MAX_POWER, turretPID.run()));

            setPower(power);
        } else {
            setPower(0);
        }
    }

    // ===================================================================
    // Compute corrected pose from vision (returns Pose if significant correction needed)
    // ===================================================================
    private Pose computeCorrectedOdometry(Pose currentPose, double robotHeading) {
        LLResult result = camera.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose();
            if (botPose != null) {
                double camX = botPose.getPosition().x;
                double camY = botPose.getPosition().y;
                double camYaw = botPose.getOrientation().getYaw();
                double turretAngle = getAngleDegrees();

                // Rotate camera position into robot frame
                double cosA = Math.cos(Math.toRadians(turretAngle));
                double sinA = Math.sin(Math.toRadians(turretAngle));
                double robotX = camX * cosA - camY * sinA;
                double robotY = camX * sinA + camY * cosA;

                double deltaX = robotX - currentPose.getX();
                double deltaY = robotY - currentPose.getY();
                double deltaHeading = Math.abs(camYaw - robotHeading);

                if (Math.hypot(deltaX, deltaY) > ODOMETRY_CORRECTION_THRESHOLD || deltaHeading > Math.toRadians(2)) {
                    return new Pose(robotX, robotY, camYaw); // Corrected pose
                }
            }
        }
        return null;
    }

    // ===================================================================
    // Utilities
    // ===================================================================
    public void setPower(double power) {
        // Respect limit switches
        if ((power > 0 && isLeftLimitPressed()) || (power < 0 && isRightLimitPressed())) {
            power = 0;
        }
        turretMotor.setPower(power);
    }

    public boolean isLeftLimitPressed() { return !limitLeft.getState(); }
    public boolean isRightLimitPressed() { return !limitRight.getState(); }
    public double getAngleDegrees() { return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE; }
    public int getEncoderTicks() { return turretMotor.getCurrentPosition(); }
    public void setShooterBlocked() { shooterBlocker.setPosition(Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION); isShooterBlocked = true; }
    public void setShooterUnBlocked() { shooterBlocker.setPosition(Constants.TurretConfig.SHOOTER_BLOCKER_ZERO_POSITION); isShooterBlocked = false; }
}
