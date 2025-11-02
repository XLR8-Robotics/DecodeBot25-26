package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.Constants;

/**
 * Subsystem for controlling the turret mechanism.
 */
public class Turret {
    private final DcMotorEx turretMotor;

    public Turret(HardwareMap hardwareMap, String turretMotorName) {
        this.turretMotor = hardwareMap.get(DcMotorEx.class, turretMotorName);

        // If the turret rotates in the wrong direction, you can reverse it by uncommenting the next line.
        // this.turretMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Call this method in your TeleOp loop to control the turret with the gamepad.
     * @param gamepad The gamepad that will control the turret.
     */
    public void update(Gamepad gamepad) {
        double turretPower = 0;

        if (gamepad.right_bumper) {
            // R1 button is pressed, rotate clockwise
            turretPower = Constants.TurretConfig.TURRET_SPEED;
        } else if (gamepad.left_bumper) {
            // L1 button is pressed, rotate counter-clockwise
            turretPower = -Constants.TurretConfig.TURRET_SPEED;
        }

        turretMotor.setPower(turretPower);
    }

    /**
     * Returns the current power of the turret motor. Useful for telemetry.
     * @return The current power of the turret motor.
     */
    public double getMotorPower() {
        return turretMotor.getPower();
    }
}
