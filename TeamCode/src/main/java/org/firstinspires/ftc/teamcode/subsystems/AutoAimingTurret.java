package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.control.PIDFController;
import com.pedropathing.control.PIDFCoefficients;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.Constants;

@Config
public class AutoAimingTurret {

    // =====================================================================
    // -------------------------- CONFIG TUNABLE ----------------------------
    // =====================================================================

    public static double P = 0.012;
    public static double I = 0.000;
    public static double D = 0.0007;
    public static double F = 0.1;

    public static double MAX_POWER = 0.8;
    public static double HOMING_POWER = 0.10;

    public static double TICKS_PER_DEG = 5.87;

    // Real mechanical limits
    public static double HARD_LEFT = 310;
    public static double HARD_RIGHT = 50;

    // Soft protection limits (keeps turret safe)
    public static double SOFT_LEFT = 300;
    public static double SOFT_RIGHT = 60;

    // Shot correction (offset based on distance)
    public static double SHOT_OFFSET_COEFFICIENT = 1.5;

    public static boolean ENABLE_TRACKING = true;
    public static boolean ENABLE_VISION_CORRECTION = true;

    // =====================================================================
    // --------------------------- HARDWARE --------------------------------
    // =====================================================================

    private final DcMotorEx motor;
    private final Servo blocker;
    private final DigitalChannel leftLimit, rightLimit;
    private final Limelight3A cam;
    private final Follower follower;

    private final PIDFController pid;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    // Target XY
    private double targetX = 0;
    private double targetY = 0;

    // Manual toggle
    private boolean blocked = true;
    private boolean prevSquare = false;

    private boolean homed = false;

    // =====================================================================
    // ---------------------------- CONSTRUCTOR -----------------------------
    // =====================================================================

    public AutoAimingTurret(HardwareMap map, Follower follower) {

        this.follower = follower;

        motor = map.get(DcMotorEx.class, Constants.HardwareConfig.TURRET_MOTOR);
        blocker = map.get(Servo.class, Constants.HardwareConfig.SHOOTER_BLOCKER);

        leftLimit = map.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_LEFT);
        rightLimit = map.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_RIGHT);

        cam = map.get(Limelight3A.class, Constants.HardwareConfig.LIMELIGHT_NAME);

        leftLimit.setMode(DigitalChannel.Mode.INPUT);
        rightLimit.setMode(DigitalChannel.Mode.INPUT);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMPORTANT → Fixes the "moves only left" issue
        //motor.setDirection(DcMotorSimple.Direction.REVERSE);

        pid = new PIDFController(new PIDFCoefficients(P, I, D, F));

        blocker.setPosition(Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION);
        blocked = true;
    }

    // =====================================================================
    // ------------------------------- HOMING -------------------------------
    // =====================================================================

    public void home() {
        if (homed) return;

        motor.setPower(-HOMING_POWER);

        if (isRightPressed()) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            homed = true;
        }
    }

    // =====================================================================
    // ------------------------ MANUAL CONTROL ------------------------------
    // =====================================================================

    public void manualUpdate(Gamepad gp) {

        double pwr = 0;

        if (gp.left_bumper && !isLeftPressed())
            pwr = +Constants.TurretConfig.TURRET_SPEED;

        else if (gp.right_bumper && !isRightPressed())
            pwr = -Constants.TurretConfig.TURRET_SPEED;

        setPower(pwr);

        if (gp.square && !prevSquare) {
            blocked = !blocked;
            blocker.setPosition(
                    blocked ?
                            Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION :
                            Constants.TurretConfig.SHOOTER_BLOCKER_ZERO_POSITION
            );
        }
        prevSquare = gp.square;
    }

    // =====================================================================
    // ---------------------------- SET TARGET ------------------------------
    // =====================================================================

    public void setTarget(double x, double y) {
        targetX = x;
        targetY = y;
    }

    // =====================================================================
    // -------------------------- MAIN UPDATE LOOP --------------------------
    // =====================================================================

    public void update() {

        if (!homed) {
            home();
            return;
        }

        Pose pose = follower.getPose();
        double headingDeg = Math.toDegrees(follower.getHeading());

        if (ENABLE_VISION_CORRECTION)
            applyVisionCorrection(pose, headingDeg);

        if (!ENABLE_TRACKING)
            return;

        // Compute desired turret angle in field frame
        double fieldAngle = Math.toDegrees(Math.atan2(
                targetY - pose.getY(),
                targetX - pose.getX()
        ));

        // Convert to turret frame (forward = 180°)
        double turretTarget = norm(fieldAngle - headingDeg + 180);

        // Shot offset correction (based on distance)
        double dist = Math.hypot(targetX - pose.getX(), targetY - pose.getY());
        turretTarget += dist * SHOT_OFFSET_COEFFICIENT;

        turretTarget = norm(turretTarget);

        // Safety — stay inside soft limits
        if (!withinSoftLimits(turretTarget)) {
            motor.setPower(0);
            return;
        }

        // PID tracking
        pid.setCoefficients(new PIDFCoefficients(P, I, D, F));
        pid.setTargetPosition(turretTarget);

        double current = getAngle();
        double error = shortestError(current, turretTarget);

        pid.updatePosition(current + error);

        double out = clamp(pid.run(), -MAX_POWER, MAX_POWER);

        setPower(out);

        sendTelemetry(turretTarget, current, out, error, dist);
    }

    // =====================================================================
    // ----------------------- VISION ODOMETRY FIX ---------------------------
    // =====================================================================

    private void applyVisionCorrection(Pose current, double headingDeg) {

        LLResult r = cam.getLatestResult();
        if (r == null || !r.isValid()) return;

        Pose3D bot = r.getBotpose();
        if (bot == null) return;

        double x = bot.getPosition().x;
        double y = bot.getPosition().y;
        double yaw = bot.getOrientation().getYaw();

        if (Math.hypot(x - current.getX(), y - current.getY()) > 0.06 ||
                Math.abs(yaw - headingDeg) > 2) {
            follower.setPose(new Pose(x, y, yaw));
        }
    }

    // =====================================================================
    // --------------------------- UTILITIES --------------------------------
    // =====================================================================

    public double getAngle() {
        return norm(motor.getCurrentPosition() / TICKS_PER_DEG);
    }

    private boolean isLeftPressed() {
        return !leftLimit.getState();
    }

    private boolean isRightPressed() {
        return !rightLimit.getState();
    }

    private void setPower(double p) {
        if (p > 0 && isLeftPressed()) p = 0;
        if (p < 0 && isRightPressed()) p = 0;

        motor.setPower(p);
    }

    private boolean withinSoftLimits(double angle) {
        if (angle >= SOFT_RIGHT && angle <= 360) return true;
        if (angle >= 0 && angle <= SOFT_LEFT) return true;
        return false;
    }

    private double shortestError(double current, double target) {
        double diff = norm(target - current);
        if (diff > 180) diff -= 360;
        return diff;
    }

    private double norm(double deg) {
        return (deg % 360 + 360) % 360;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // =====================================================================
    // ----------------------------- TELEMETRY ------------------------------
    // =====================================================================

    private void sendTelemetry(double target, double current, double out, double err, double dist) {
        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Turret Angle", current);
        packet.put("Turret Target", target);
        packet.put("Error", err);
        packet.put("Power", out);
        packet.put("Distance to Target", dist);

        packet.put("Left Limit", isLeftPressed());
        packet.put("Right Limit", isRightPressed());
        packet.put("Homed", homed);
        packet.put("Tracking Enabled", ENABLE_TRACKING);
        packet.put("Vision Enabled", ENABLE_VISION_CORRECTION);

        dashboard.sendTelemetryPacket(packet);
    }
}
