package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.config.Constants;

public class Shooter {
    private final DcMotorEx shooterMotor;
    private final Servo hoodServo;

    public Shooter(HardwareMap hardwareMap) {
        this.shooterMotor = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.SHOOTER_MOTOR);
        this.hoodServo = hardwareMap.get(Servo.class, Constants.HardwareConfig.HOOD_SERVO);

        // If the shooter motor spins in the wrong direction, you can reverse it by uncommenting the next line.
        // this.shooterMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Call this method in your TeleOp loop to control the shooter with the gamepad.
     * @param gamepad The gamepad that will control the shooter.
     */
    public void update(Gamepad gamepad) {
        // --- Shooter Motor Control ---
        if (gamepad.x) {
            shooterMotor.setPower(Constants.ShooterConfig.SHOOTER_SPEED);
        } else {
            shooterMotor.setPower(0);
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
}
