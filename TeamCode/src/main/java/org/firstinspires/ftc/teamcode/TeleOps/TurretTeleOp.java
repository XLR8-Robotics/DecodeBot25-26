package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "Turret Control", group = "TeleOp")
public class TurretTeleOp extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the robot, which in turn initializes the turret
        robot = new Robot();
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update the turret based on gamepad 1 input
            robot.turret.update(gamepad1);

            // Display the turret motor power on the driver station
            telemetry.addData("Turret Power", robot.turret.getMotorPower());
            telemetry.update();
        }
    }
}
