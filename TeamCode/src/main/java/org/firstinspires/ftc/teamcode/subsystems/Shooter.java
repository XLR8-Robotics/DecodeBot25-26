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

    public Shooter(HardwareMap hardwareMap) {
        this.shooterMotor = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.SHOOTER_MOTOR);
        this.hoodServo = hardwareMap.get(Servo.class, Constants.HardwareConfig.HOOD_SERVO);
        this.isRunning = false;

        // Set initial hood position
        hoodServo.setPosition(Constants.ShooterConfig.HOOD_DEFAULT_POSITION);

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
            setPower(Constants.ShooterConfig.SHOOTER_SPEED);
            isRunning = true;
        } else {
            setPower(0);
            isRunning = false;
        }
    }

    public void setPower(double power) {
        shooterMotor.setPower(power);
        isRunning = power > 0;
    }

    public void setHoodPosition(double position) {
        hoodServo.setPosition(position);
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
