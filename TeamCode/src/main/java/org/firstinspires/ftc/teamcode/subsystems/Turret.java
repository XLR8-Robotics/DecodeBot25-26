package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TuningConfig;

import java.util.HashMap;
import java.util.Map;

/**
 * Centralized, configurable mapping for DRIVETRAIN controls only.
 * Change your TeleOp drive scheme here in one place.
 */
public class Turret {
    private final DcMotorEx turretMotor;

    public Turret(HardwareMap hardwareMap, String turretMotorName) {
        this.turretMotor = hardwareMap.get(DcMotorEx.class, turretMotorName);
    }

}