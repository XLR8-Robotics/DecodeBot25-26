package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.config.Constants;

import java.util.Arrays;
import java.util.List;

/**
 * Subsystem for controlling the shooter mechanism, including the shooter wheel and hood.
 */
public class Shooter {
    private final DcMotorEx shooterMotor;
    private final Servo hoodServo;
    private boolean isRunning;

    private final List<Double> hoodPositions = Arrays.asList(
            Constants.ShooterConfig.HOOD_MIN,
            Constants.ShooterConfig.HOOD_POS_2,
            Constants.ShooterConfig.HOOD_POS_3,
            Constants.ShooterConfig.HOOD_POS_4,
            Constants.ShooterConfig.HOOD_CENTER,
            Constants.ShooterConfig.HOOD_POS_6,
            Constants.ShooterConfig.HOOD_POS_7,
            Constants.ShooterConfig.HOOD_POS_8,
            Constants.ShooterConfig.HOOD_MAX
    );
    private int currentHoodPositionIndex = 4; // Start at center position
    private boolean previousDpadUpState = false;
    private boolean previousDpadDownState = false;

    public Shooter(HardwareMap hardwareMap) {
        this.shooterMotor = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.SHOOTER_MOTOR);
        this.hoodServo = hardwareMap.get(Servo.class, Constants.HardwareConfig.HOOD_SERVO);
        this.isRunning = false;

        // Set initial hood position
        setHoodPosition(hoodPositions.get(currentHoodPositionIndex));

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

        // --- Hood Control ---
        boolean currentDpadUpState = gamepad.dpad_up;
        boolean currentDpadDownState = gamepad.dpad_down;

        if (currentDpadUpState && !previousDpadUpState) {
            // Move to the next hood position
            currentHoodPositionIndex = Math.min(hoodPositions.size() - 1, currentHoodPositionIndex + 1);
            setHoodPosition(hoodPositions.get(currentHoodPositionIndex));
        } else if (currentDpadDownState && !previousDpadDownState) {
            // Move to the previous hood position
            currentHoodPositionIndex = Math.max(0, currentHoodPositionIndex - 1);
            setHoodPosition(hoodPositions.get(currentHoodPositionIndex));
        }

        previousDpadUpState = currentDpadUpState;
        previousDpadDownState = currentDpadDownState;
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
