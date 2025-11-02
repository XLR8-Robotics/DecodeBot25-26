package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "Intake Control", group = "TeleOp")
public class IntakeTeleOp extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the robot, which in turn initializes all subsystems
        robot = new Robot();
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update the intake based on gamepad 1 triggers
            robot.intake.update(gamepad1);

            // Display the intake motor power on the driver station
            telemetry.addData("Intake Power", robot.intake.getMotorPower());
            telemetry.update();
        }
    }
}
