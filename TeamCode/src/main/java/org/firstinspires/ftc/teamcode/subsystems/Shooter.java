package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.config.Constants;

import java.util.Arrays;

/**
 * Subsystem for controlling the shooter mechanism, including the shooter wheel and hood.
 */
public class Shooter {
    private final DcMotorEx shooterMotor;
    private final Servo hoodServo;

    // An array to hold the different hood positions.
    private final double[] hoodPositions;
    // An index to track which state (position) we are currently in.
    private int hoodPositionIndex;

    // "Ready" flags to ensure one state change per button press.
    private boolean isDpadUpReady = true;
    private boolean isDpadDownReady = true;

    public Shooter(HardwareMap hardwareMap) {
        this.shooterMotor = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.SHOOTER_MOTOR);
        this.hoodServo = hardwareMap.get(Servo.class, Constants.HardwareConfig.HOOD_SERVO);

        // Define the cycle of hood positions.
        this.hoodPositions = new double[]{
                Constants.ShooterConfig.HOOD_DOWN_POSITION,
                Constants.ShooterConfig.HOOD_DEFAULT_POSITION,
                Constants.ShooterConfig.HOOD_UP_POSITION
        };
        Arrays.sort(this.hoodPositions);

        // Start in the middle state.
        this.hoodPositionIndex = 1;
    }

    /**
     * This is the main control method, called repeatedly in the TeleOp loop.
     * @param gamepad The gamepad that will control the shooter.
     */
    public void update(Gamepad gamepad) {
        // --- Shooter Motor Control ---
        if (gamepad.x) {
            shooterMotor.setPower(Constants.ShooterConfig.SHOOTER_SPEED);
        } else {
            shooterMotor.setPower(0);
        }

        // --- State Cycling Hood Servo Control ---

        // If D-pad Up is pressed AND the button is "ready"...
        if (gamepad.dpad_up && isDpadUpReady) {
            // ...change the state by cycling up through the positions.
            hoodPositionIndex = (hoodPositionIndex + 1) % hoodPositions.length;
            // ...and immediately set the button to "not ready" to prevent another change.
            isDpadUpReady = false;
        }

        // If D-pad Down is pressed AND the button is "ready"...
        if (gamepad.dpad_down && isDpadDownReady) {
            // ...change the state by cycling down through the positions.
            hoodPositionIndex = (hoodPositionIndex - 1 + hoodPositions.length) % hoodPositions.length;
            // ...and immediately set the button to "not ready".
            isDpadDownReady = false;
        }

        // If the D-pad Up button is released, it becomes "ready" to be pressed again.
        if (!gamepad.dpad_up) {
            isDpadUpReady = true;
        }

        // If the D-pad Down button is released, it becomes "ready" to be pressed again.
        if (!gamepad.dpad_down) {
            isDpadDownReady = true;
        }

        // Always set the servo to the currently tracked state.
        hoodServo.setPosition(hoodPositions[hoodPositionIndex]);
    }

    /**
     * Returns the current power of the shooter motor for telemetry.
     */
    public double getMotorPower() {
        return shooterMotor.getPower();
    }

    /**
     * Returns the current target position of the hood servo for telemetry.
     */
    public double getServoPosition() {
        return hoodPositions[hoodPositionIndex];
    }
}
