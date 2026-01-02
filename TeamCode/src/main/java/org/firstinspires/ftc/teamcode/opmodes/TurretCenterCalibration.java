package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.config.Constants;

@TeleOp(name = "Turret Center Calibration", group = "Calibration")
public class TurretCenterCalibration extends LinearOpMode {

    private DcMotorEx turretMotor;
    private DigitalChannel rightLimit, leftLimit;

    @Override
    public void runOpMode() {

        turretMotor = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.TURRET_MOTOR);
        rightLimit = hardwareMap.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_RIGHT);
        leftLimit  = hardwareMap.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_LEFT);

        rightLimit.setMode(DigitalChannel.Mode.INPUT);
        leftLimit.setMode(DigitalChannel.Mode.INPUT);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Turret Center Calibration");
        telemetry.addLine("Will home to RIGHT limit on start");
        telemetry.update();

        waitForStart();

        // ================= HOME TO RIGHT =================
        while (opModeIsActive() && rightLimit.getState()) {
            turretMotor.setPower(-0.15);
            telemetry.addLine("Homing to RIGHT...");
            telemetry.update();
        }

        turretMotor.setPower(0);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ================= MANUAL ADJUST =================
        while (opModeIsActive()) {

            double power = 0;

            if (gamepad1.left_bumper && leftLimit.getState()) {
                power = +0.2; // move LEFT
            } else if (gamepad1.right_bumper && rightLimit.getState()) {
                power = -0.2; // move RIGHT
            }

            turretMotor.setPower(power);

            telemetry.addLine("Move turret to PHYSICAL CENTER");
            telemetry.addLine("LEFT BUMPER  = move left");
            telemetry.addLine("RIGHT BUMPER = move right");
            telemetry.addLine("");
            telemetry.addData("Encoder Ticks (CENTER OFFSET)", turretMotor.getCurrentPosition());
            telemetry.addLine("");
            telemetry.addLine("Write this number down!");
            telemetry.update();
        }
    }
}
