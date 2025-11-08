package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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

    public Turret(HardwareMap hardwareMap) {
        this.turretMotor = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.TURRET_MOTOR);
        this.shooterBlocker = hardwareMap.get(Servo.class, Constants.HardwareConfig.SHOOTER_BLOCKER);
        this.turretLimitLeft = hardwareMap.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_LEFT);
        this.turretLimitRight = hardwareMap.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_RIGHT);

        // Set the mode to input
        turretLimitLeft.setMode(DigitalChannel.Mode.INPUT);
        turretLimitRight.setMode(DigitalChannel.Mode.INPUT);
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
