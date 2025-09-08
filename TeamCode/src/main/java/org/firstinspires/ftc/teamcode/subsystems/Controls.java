package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.TuningConfig;

import java.util.HashMap;
import java.util.Map;

/**
 * Centralized, configurable mapping for DRIVETRAIN controls only.
 * Change your TeleOp drive scheme here in one place.
 */
public class Controls {
    public enum Action {
        DRIVE_PRECISION,
    }

    public enum Analog {
        DRIVE_FORWARD,
        DRIVE_STRAFE,
        DRIVE_TURN,
    }

    public interface BoolSource { boolean get(Gamepad gp); }
    public interface AnalogSource { double get(Gamepad gp); }

    private final Map<Action, BoolSource> boolBindings = new HashMap<>();
    private final Map<Analog, AnalogSource> analogBindings = new HashMap<>();

    private double analogDeadband = TuningConfig.ANALOG_DEADBAND;

    public Controls() {
        setDefaultBindings();
    }

    public void setAnalogDeadband(double db) { this.analogDeadband = db; }

    public Controls bind(Action action, BoolSource source) {
        boolBindings.put(action, source);
        return this;
    }

    public Controls bind(Analog analog, AnalogSource source) {
        analogBindings.put(analog, source);
        return this;
    }

    public void setDefaultBindings() {
        // Buttons
        bind(Action.DRIVE_PRECISION, gp -> gp.left_bumper);

        // Axes
        bind(Analog.DRIVE_FORWARD, gp -> applyDeadband(-gp.left_stick_y));
        bind(Analog.DRIVE_STRAFE, gp -> applyDeadband(-gp.left_stick_x));
        bind(Analog.DRIVE_TURN, gp -> applyDeadband(-gp.right_stick_x));
    }

    public boolean get(Action action, Gamepad gp) {
        BoolSource src = boolBindings.get(action);
        return src != null && src.get(gp);
    }

    public double get(Analog analog, Gamepad gp) {
        AnalogSource src = analogBindings.get(analog);
        return src != null ? src.get(gp) : 0.0;
    }

    private double applyDeadband(double v) {
        return Math.abs(v) < analogDeadband ? 0.0 : v;
    }
}


