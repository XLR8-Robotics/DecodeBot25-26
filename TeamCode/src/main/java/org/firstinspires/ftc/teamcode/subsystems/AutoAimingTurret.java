package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.control.PIDFController;
import com.pedropathing.control.PIDFCoefficients;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.Constants;

@Config
public class AutoAimingTurret {

    // ===================== LIVE TUNING =====================
    public static double P = 0.05;
    public static double I = 0.01;
    public static double D = 0.01;
    public static double F = 0.5;

    public static double MAX_POWER = 0.9;
    public static double STATIC_FF = 0.006;
    public static double ANGLE_DEADBAND_DEG = 0.75;

    public static double SMOOTH_MAX_DELTA_FAR = 0.18;
    public static double SMOOTH_MAX_DELTA_NEAR = 0.05;
    public static double LARGE_ERROR_DEG = 25.0;

    public static double TICKS_PER_DEG = 5.87;

    // ===================== TURRET LIMITS =====================
    private static final double RAW_RIGHT_LIMIT_TICKS = -394;
    private static final double RAW_LEFT_LIMIT_TICKS  = 394;

    // ===================== VISION FILTERING =====================
    private static final long VISION_UPDATE_PERIOD_MS = 120; // ~8 Hz
    private static final double MAX_VISION_JUMP_IN = 12.0;

    // ===================== HARDWARE =====================
    private final DcMotorEx motor;
    private final Servo blocker;
    private final DigitalChannel leftLimit, rightLimit;

    private final Limelight limelightHardware;
    public final Follower follower;

    private final PIDFController pid;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    // ===================== STATE =====================
    private Pose towerPosition;

    private double lastPower = 0.0;
    public double turretTargetTicks = 0.0;
    public double targetFieldDeg;

    private double centerOffsetTicks = 0.0;
    private double RIGHT_LIMIT_TICKS;
    private double LEFT_LIMIT_TICKS;

    private long lastVisionUpdateMs = 0;

    private boolean isShooterBlocked = true;
    private boolean previousSquareButtonState = false;

    // ===================== CONSTRUCTOR =====================
    public AutoAimingTurret(HardwareMap map, Follower follower) {
        this.follower = follower;

        limelightHardware = new Limelight(map);
        limelightHardware.start();

        motor = map.get(DcMotorEx.class, Constants.HardwareConfig.TURRET_MOTOR);
        blocker = map.get(Servo.class, Constants.HardwareConfig.SHOOTER_BLOCKER);

        leftLimit = map.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_LEFT);
        rightLimit = map.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_RIGHT);

        leftLimit.setMode(DigitalChannel.Mode.INPUT);
        rightLimit.setMode(DigitalChannel.Mode.INPUT);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        centerOffsetTicks = motor.getCurrentPosition();
        RIGHT_LIMIT_TICKS = RAW_RIGHT_LIMIT_TICKS - centerOffsetTicks;
        LEFT_LIMIT_TICKS  = RAW_LEFT_LIMIT_TICKS - centerOffsetTicks;

        pid = new PIDFController(new PIDFCoefficients(P, I, D, F));

        blocker.setPosition(Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION);
    }

    // ===================== LIVE TUNING SETTERS =====================
    public void setPIDFValues(double p, double i, double d, double f) {
        P = p;
        I = i;
        D = d;
        F = f;
    }

    public void setMotionTuning(double maxPower,
                                double staticFf,
                                double deadbandDeg,
                                double smoothFar,
                                double smoothNear,
                                double largeErrorDeg) {

        MAX_POWER = maxPower;
        STATIC_FF = staticFf;
        ANGLE_DEADBAND_DEG = deadbandDeg;
        SMOOTH_MAX_DELTA_FAR = smoothFar;
        SMOOTH_MAX_DELTA_NEAR = smoothNear;
        LARGE_ERROR_DEG = largeErrorDeg;
    }


    // ===================== ANGLE MATH =====================
    public double findingAngle(Pose robotPose, Pose towerPose) {
        double dx = towerPose.getX() - robotPose.getX();
        double dy = towerPose.getY() - robotPose.getY();
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // ===================== UPDATE LOOP =====================
    public void update() {
        limelightHardware.updateResult();

        Pose robotPose = follower.getPose();

        // Vision only corrects odometry
        correctPosition(robotPose);

        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());
        targetFieldDeg = findingAngle(robotPose, towerPosition);

        double turretTargetDeg =
                normalizeAngle(targetFieldDeg - robotHeadingDeg);

        turretTargetTicks = turretTargetDeg * TICKS_PER_DEG;
        turretTargetTicks = clamp(turretTargetTicks, RIGHT_LIMIT_TICKS, LEFT_LIMIT_TICKS);

        double currentTicks = motor.getCurrentPosition() - centerOffsetTicks;
        double error = turretTargetTicks - currentTicks;

        if (Math.abs(error) < ANGLE_DEADBAND_DEG * TICKS_PER_DEG) {
            setPower(0);
            lastPower = 0;
            return;
        }

        pid.setCoefficients(new PIDFCoefficients(P, I, D, F));
        pid.setTargetPosition(turretTargetTicks);
        pid.updatePosition(currentTicks);

        double power = clamp(pid.run(), -MAX_POWER, MAX_POWER);
        if (power != 0) power += Math.signum(power) * STATIC_FF;

        power = smoothPower(
                power,
                Math.abs(error) > LARGE_ERROR_DEG * TICKS_PER_DEG
                        ? SMOOTH_MAX_DELTA_FAR
                        : SMOOTH_MAX_DELTA_NEAR
        );

        setPower(power);
    }

    // ===================== VISION CORRECTION =====================
    private void correctPosition(Pose odoPose) {
        LLResult ll = limelightHardware.getLatestResult();
        long now = System.currentTimeMillis();

        if (!isVisionValid(ll)) return;
        if (now - lastVisionUpdateMs < VISION_UPDATE_PERIOD_MS) return;

        Pose3D p = ll.getBotpose();

        Pose visionPose = new Pose(
                p.getPosition().x * 39.37 + 72,
                p.getPosition().y * 39.37 + 72,
                odoPose.getHeading()
        );

        if (visionPose.distanceFrom(odoPose) > MAX_VISION_JUMP_IN) return;

        follower.setPose(visionPose);
        lastVisionUpdateMs = now;
    }

    private boolean isVisionValid(LLResult ll) {
        if (ll == null) return false;
        if (!ll.isValid()) return false;
        if (ll.getBotpose() == null) return false;

        Pose3D p = ll.getBotpose();
        return !(Math.abs(p.getPosition().x) < 1e-3 &&
                Math.abs(p.getPosition().y) < 1e-3);
    }

    // ===================== SAFETY =====================
    private void setPower(double p) {
        double pos = motor.getCurrentPosition() - centerOffsetTicks;

        if (p > 0 && pos >= LEFT_LIMIT_TICKS)  p = 0;
        if (p < 0 && pos <= RIGHT_LIMIT_TICKS) p = 0;

        if (isLeftPressed() && p > 0)  p = 0;
        if (isRightPressed() && p < 0) p = 0;

        motor.setPower(p);
    }

    private boolean isLeftPressed() {
        return !leftLimit.getState();
    }

    private boolean isRightPressed() {
        return !rightLimit.getState();
    }

    private double smoothPower(double target, double maxDelta) {
        double delta = clamp(target - lastPower, -maxDelta, maxDelta);
        lastPower += delta;
        return lastPower;
    }

    // ===================== SETTERS =====================
    public void setTowerPosition(Pose pose) {
        towerPosition = pose;
    }

    public void changeBlockingPose(Gamepad gamepad){
        boolean currentSquareButtonState = gamepad.square;
        if (currentSquareButtonState && !previousSquareButtonState)
        {
            isShooterBlocked = !isShooterBlocked; // Toggle the state
            if (isShooterBlocked)
            {
                setShooterUnBlocked();
            } else
            {
                setShooterUnBlocked();
            }
        }
        previousSquareButtonState = currentSquareButtonState;
    }

    public void setShooterBlocked(){
        blocker.setPosition(Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION);
    }
    public void setShooterUnBlocked(){
        blocker.setPosition(Constants.TurretConfig.SHOOTER_BLOCKER_ZERO_POSITION);
    }
}
