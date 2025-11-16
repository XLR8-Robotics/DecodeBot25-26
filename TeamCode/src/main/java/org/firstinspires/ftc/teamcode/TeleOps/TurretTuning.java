package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "Turret Tuning", group = "Tuning")
public class TurretTuning extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Info", "Press START to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            robot.turret.manualUpdate(gamepad1);

            telemetry.addData("Turret Encoder Ticks", robot.turret.getEncoderTicks());
            telemetry.addData("Left Limit Switch Pressed", robot.turret.isLeftLimitPressed());
            telemetry.addData("Right Limit Switch Pressed", robot.turret.isRightLimitPressed());
            telemetry.update();
        }
    }
}
