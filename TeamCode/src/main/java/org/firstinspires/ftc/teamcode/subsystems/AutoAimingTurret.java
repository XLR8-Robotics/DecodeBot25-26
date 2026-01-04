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

    public static double TICKS_PER_DEG = 5.87;

    // Turret physical limits (measured)
    private static final double RAW_RIGHT_LIMIT_TICKS = -394;
    private static final double RAW_LEFT_LIMIT_TICKS  = 394;
    public final Limelight limelightHardware;
    public Pose limeLightPositon;

    // ===================== HARDWARE =====================
    private final DcMotorEx motor;
    private final Servo blocker;
    private final DigitalChannel leftLimit, rightLimit;
    public final Follower follower;
    private final PIDFController pid;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private Pose towerPosition;

    private boolean homed = false;
    private boolean blocked = true;
    private boolean prevSquare = false;

    private double lastPower = 0.0;
    public double turretTargetTicks = 0.0;
    public double targetFieldDeg;

    // Center offset (calculated at startup)
    private double centerOffsetTicks = 0.0;

    // Limits relative to center
    private double RIGHT_LIMIT_TICKS;
    private double LEFT_LIMIT_TICKS;

    // ===================== CONSTRUCTOR =====================
    public AutoAimingTurret(HardwareMap map, Follower follower) {
        this.follower = follower;

        limelightHardware = new Limelight(map);

        motor = map.get(DcMotorEx.class, Constants.HardwareConfig.TURRET_MOTOR);
        blocker = map.get(Servo.class, Constants.HardwareConfig.SHOOTER_BLOCKER);

        leftLimit = map.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_LEFT);
        rightLimit = map.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_RIGHT);

        leftLimit.setMode(DigitalChannel.Mode.INPUT);
        rightLimit.setMode(DigitalChannel.Mode.INPUT);

        // ================== RESET ENCODER ==================
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define center offset (we treat current position as center)
        centerOffsetTicks = motor.getCurrentPosition();

        // Calculate limits relative to center
        RIGHT_LIMIT_TICKS = RAW_RIGHT_LIMIT_TICKS - centerOffsetTicks;
        LEFT_LIMIT_TICKS  = RAW_LEFT_LIMIT_TICKS - centerOffsetTicks;

        pid = new PIDFController(new PIDFCoefficients(P, I, D, F));

        // Start blocked
        blocker.setPosition(Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION);
        blocked = true;
    }

    // ===================== ANGLE MATH =====================
    public double findingAngle(Pose robotPose, Pose towerPose) {
        double dx = towerPose.getX() - robotPose.getX();
        double dy = towerPose.getY() - robotPose.getY();
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    public void setPIDFValues(double p, double i, double d, double f)
    {
            P = p;
            I = i;
            D = d;
            F = f;
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
        Pose pose = follower.getPose();
        Pose3D llPose = limelightHardware.getLatestResult().getBotpose();
        limeLightPositon = new Pose(llPose.getPosition().x + 72, llPose.getPosition().y+72);

        double robotHeadingDeg = Math.toDegrees(pose.getHeading());
        targetFieldDeg = findingAngle(pose, towerPosition);

        // Turret-relative angle
        double turretTargetDeg = normalizeAngle(targetFieldDeg - robotHeadingDeg);

        // Convert to ticks
        turretTargetTicks = turretTargetDeg * TICKS_PER_DEG;

        // Clamp to safe turret limits
        turretTargetTicks = clamp(turretTargetTicks, RIGHT_LIMIT_TICKS, LEFT_LIMIT_TICKS);

        // Read current ticks relative to center
        double currentTicks = motor.getCurrentPosition() - centerOffsetTicks;
        double error = turretTargetTicks - currentTicks;

        // Deadband
        if (Math.abs(error) < ANGLE_DEADBAND_DEG * TICKS_PER_DEG) {
            setPower(0);
            lastPower = 0;
            return;
        }

        // PID
        pid.setCoefficients(new PIDFCoefficients(P, I, D, F));
        pid.setTargetPosition(turretTargetTicks);
        pid.updatePosition(currentTicks);

        double power = clamp(pid.run(), -MAX_POWER, MAX_POWER);
        if (power != 0) power += Math.signum(power) * STATIC_FF;
        power = smoothPower(power, 0.03);

        setPower(power);
        sendTelemetry(currentTicks, power, error);
    }

    // ===================== SAFETY =====================
    private void setPower(double p) {
        double pos = motor.getCurrentPosition() - centerOffsetTicks;
        if (p > 0 && pos >= LEFT_LIMIT_TICKS)  p = 0;
        if (p < 0 && pos <= RIGHT_LIMIT_TICKS) p = 0;
        if (isLeftPressed() && p > 0) p = 0;
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

    // ===================== TELEMETRY =====================
    private void sendTelemetry(double current, double power, double error) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Turret Ticks", current);
        packet.put("Target Ticks", turretTargetTicks);
        packet.put("Error", error);
        packet.put("Power", power);
        packet.put("Homed", homed);
        dashboard.sendTelemetryPacket(packet);
    }

    // ===================== SETTERS =====================
    public void setTowerPosition(Pose pose) {
        towerPosition = pose;
    }
}
