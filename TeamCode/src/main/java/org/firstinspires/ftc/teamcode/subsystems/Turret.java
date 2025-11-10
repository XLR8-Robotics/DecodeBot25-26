package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.Constants;

/**
 * Subsystem for controlling the turret mechanism.
 */
public class Turret {
    private final DcMotorEx turretMotor;
    private final Servo shooterBlocker;
    private final DigitalChannel turretLimitLeft;
    private final DigitalChannel turretLimitRight;

    private boolean isShooterBlocked = true;
    private boolean previousSquareButtonState = false;

    public Turret(HardwareMap hardwareMap) {
        this.turretMotor = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.TURRET_MOTOR);
        this.shooterBlocker = hardwareMap.get(Servo.class, Constants.HardwareConfig.SHOOTER_BLOCKER);
        this.turretLimitLeft = hardwareMap.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_LEFT);
        this.turretLimitRight = hardwareMap.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_RIGHT);

        // Set the mode to input
        turretLimitLeft.setMode(DigitalChannel.Mode.INPUT);
        turretLimitRight.setMode(DigitalChannel.Mode.INPUT);

        // Reset the encoder and set the motor to run using encoders.
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Start with the shooter blocker in the blocking position
        setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION);
    }

    public void update(Gamepad gamepad) {
        double turretPower = 0;
        if (gamepad.left_bumper) {
            turretPower = -Constants.TurretConfig.TURRET_SPEED;
        } else if (gamepad.right_bumper) {
            turretPower = Constants.TurretConfig.TURRET_SPEED;
        }

        // Apply limit switch logic
        if (isLeftLimitPressed() && turretPower < 0) {
            turretPower = 0;
        }
        if (isRightLimitPressed() && turretPower > 0) {
            turretPower = 0;
        }

        setPower(turretPower);

        // Shooter blocker toggle with square button
        boolean currentSquareButtonState = gamepad.square;
        if (currentSquareButtonState && !previousSquareButtonState) {
            isShooterBlocked = !isShooterBlocked; // Toggle the state
            if (isShooterBlocked) {
                setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION);
            } else {
                setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_ZERO_POSITION);
            }
        }
        previousSquareButtonState = currentSquareButtonState;
    }

    public void setPower(double power) {
        turretMotor.setPower(power);
    }

    public void setShooterBlockerPosition(double position) {
        shooterBlocker.setPosition(position);
    }

    public double getMotorPower() {
        return turretMotor.getPower();
    }

    public double getShooterBlockerPosition() {
        return shooterBlocker.getPosition();
    }

    /**
     * Calculates the current angle of the turret in degrees.
     * @return The turret's angle.
     */
    public double getAngle() {
        double ticks = turretMotor.getCurrentPosition();
        double revolutions = ticks / Constants.TurretConfig.TURRET_TICKS_PER_REV;
        double turretRotations = revolutions / Constants.TurretConfig.TURRET_GEAR_RATIO;
        return turretRotations * 360;
    }

    /**
     * Checks if the left magnetic limit switch is pressed.
     * @return true if the switch is pressed, false otherwise.
     */
    public boolean isLeftLimitPressed() {
        // The getState() method returns true if the switch is not pressed (open)
        // and false if it is pressed (closed by the magnet). We invert it for intuitive use.
        return !turretLimitLeft.getState();
    }

    /**
     * Checks if the right magnetic limit switch is pressed.
     * @return true if the switch is pressed, false otherwise.
     */
    public boolean isRightLimitPressed() {
        return !turretLimitRight.getState();
    }
}
