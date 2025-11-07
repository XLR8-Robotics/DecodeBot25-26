package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.config.Constants;

/**
 * Subsystem for controlling the shooter mechanism, including the shooter wheel and hood.
 */
public class Shooter {
    private final DcMotorEx shooterMotor;
    private final Servo hoodServo;
    private boolean isRunning;

    public Shooter(HardwareMap hardwareMap, String shooterMotorName, String hoodServoName) {
        this.shooterMotor = hardwareMap.get(DcMotorEx.class, shooterMotorName);
        this.hoodServo = hardwareMap.get(Servo.class, hoodServoName);
        this.isRunning = false;

        // If the shooter motor spins in the wrong direction, you can reverse it by uncommenting the next line.
        // this.shooterMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Call this method in your TeleOp loop to control the shooter with the gamepad.
     * @param gamepad The gamepad that will control the shooter.
     */
    public void update(Gamepad gamepad) {
        // --- Shooter Motor Control ---
        if (gamepad.cross) {
            shooterMotor.setPower(Constants.ShooterConfig.SHOOTER_SPEED);
            isRunning = true;
        } else {
            shooterMotor.setPower(0);
            isRunning = false;
        }

        // --- Hood Servo Control ---
        if (gamepad.dpad_up) {
            hoodServo.setPosition(Constants.ShooterConfig.HOOD_UP_POSITION);
        } else if (gamepad.dpad_down) {
            hoodServo.setPosition(Constants.ShooterConfig.HOOD_DOWN_POSITION);
        } else {
            hoodServo.setPosition(Constants.ShooterConfig.HOOD_DEFAULT_POSITION);
        }
    }

    /**
     * Returns the current power of the shooter motor. Useful for telemetry.
     * @return The current power of the shooter motor.
     */
    public double getMotorPower() {
        return shooterMotor.getPower();
    }

    /**
     * Returns the current position of the hood servo. Useful for telemetry.
     * @return The current position of the hood servo.
     */
    public double getServoPosition() {
        return hoodServo.getPosition();
    }

    public boolean isRunning() {return isRunning;
    }
}
