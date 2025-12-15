package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.control.PIDFController;
import com.pedropathing.control.PIDFCoefficients;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Constants;

@Config
public class AutoAimingTurret {

    // =====================================================================
    // -------------------------- LIVE TUNABLE -----------------------------
    // =====================================================================
    public static double P = 0.006;
    public static double I = 0.0;
    public static double D = 0.00025;
    public static double F = 0.0;

    public static double MAX_POWER = 0.6;
    public static double STATIC_FF = 0.06;
    public static double ANGLE_DEADBAND_DEG = 0.75;

    public static double TICKS_PER_DEG = 5.87;

    // Soft limits
    public static double SOFT_LEFT = 300;
    public static double SOFT_RIGHT = 60;

    // =====================================================================
    // --------------------------- HARDWARE --------------------------------
    // =====================================================================
    private final DcMotorEx motor;
    private final Servo blocker;
    private final DigitalChannel leftLimit, rightLimit;
    private final Follower follower;

    private final PIDFController pid;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private double fieldAngle = 0.0; // Desired field angle
    private boolean blocked = true;
    private boolean prevSquare = false;

    private boolean homed = false;
    private double lastPower = 0.0; // For slew smoothing

    // =====================================================================
    // --------------------------- CONSTRUCTOR -----------------------------
    // =====================================================================
    public AutoAimingTurret(HardwareMap map, Follower follower) {
        this.follower = follower;

        motor = map.get(DcMotorEx.class, Constants.HardwareConfig.TURRET_MOTOR);
        blocker = map.get(Servo.class, Constants.HardwareConfig.SHOOTER_BLOCKER);

        leftLimit = map.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_LEFT);
        rightLimit = map.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_RIGHT);

        leftLimit.setMode(DigitalChannel.Mode.INPUT);
        rightLimit.setMode(DigitalChannel.Mode.INPUT);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pid = new PIDFController(new PIDFCoefficients(P, I, D, F));

        blocker.setPosition(Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION);
        blocked = true;
    }

    // =====================================================================
    // ------------------------ HOMING LOGIC --------------------------------
    // =====================================================================
    public void home() {
        if (homed) return;

        motor.setPower(-0.10);

        if (isRightPressed()) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            homed = true;
        }
    }

    // =====================================================================
    // ------------------------ MANUAL CONTROL -----------------------------
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
    // ----------------------- SET FIELD ANGLE -----------------------------
    // =====================================================================
    public void setFieldAngle(double deg) {
        fieldAngle = deg;
    }

    public double getFieldAngle() {
        return fieldAngle;
    }

    // =====================================================================
    // ----------------------- MAIN UPDATE LOOP -----------------------------
    // =====================================================================
    public void update() {

        if (!homed) {
            home();
            return;
        }

        Pose pose = follower.getPose();
        double headingDeg = Math.toDegrees(pose.getHeading());

        // Desired turret angle in turret frame
        double turretTarget = wrap(fieldAngle - headingDeg + 180);

        double current = getAngle();
        double error = shortestError(current, turretTarget);

        // Deadband: stop motor if close enough
        if (Math.abs(error) < ANGLE_DEADBAND_DEG) {
            setPower(0);
            lastPower = 0;
            return;
        }

        // PID coefficients from live dashboard
        pid.setCoefficients(new PIDFCoefficients(P, I, D, F));
        pid.setTargetPosition(turretTarget);

        pid.updatePosition(current);
        double power = clamp(pid.run(), -MAX_POWER, MAX_POWER);

        // Add static feedforward
        if (power != 0) power += Math.signum(power) * STATIC_FF;

        // Optional slew smoothing
        power = smoothPower(power, 0.03);

        setPower(power);

        sendTelemetry(turretTarget, current, power, error);
    }

    // =====================================================================
    // ------------------------- UTILITIES --------------------------------
    // =====================================================================
    public double getAngle() {
        return wrap(motor.getCurrentPosition() / TICKS_PER_DEG);
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

    private double shortestError(double current, double target) {
        double diff = wrap(target - current);
        if (diff > 180) diff -= 360;
        return diff;
    }

    private double wrap(double deg) {
        return (deg % 360 + 360) % 360;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double smoothPower(double target, double maxDelta) {
        double delta = clamp(target - lastPower, -maxDelta, maxDelta);
        lastPower += delta;
        return lastPower;
    }

    // =====================================================================
    // ------------------------- TELEMETRY ---------------------------------
    // =====================================================================
    private void sendTelemetry(double target, double current, double power, double error) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Turret Angle", current);
        packet.put("Turret Target", target);
        packet.put("Error", error);
        packet.put("Motor Power", power);
        packet.put("P", P);
        packet.put("I", I);
        packet.put("D", D);
        packet.put("Deadband", ANGLE_DEADBAND_DEG);
        packet.put("Static FF", STATIC_FF);
        packet.put("Homed", homed);

        dashboard.sendTelemetryPacket(packet);
    }
}
