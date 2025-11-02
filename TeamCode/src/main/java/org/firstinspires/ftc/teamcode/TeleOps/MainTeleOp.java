package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends LinearOpMode {

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
            // Update all subsystems based on gamepad 1 input
            robot.turret.update(gamepad1);
            robot.intake.update(gamepad1);
            robot.shooter.update(gamepad1);

            // Display the motor power and servo position for all subsystems
            telemetry.addData("Turret Power", robot.turret.getMotorPower());
            telemetry.addData("Intake Power", robot.intake.getMotorPower());
            telemetry.addData("Shooter Power", robot.shooter.getMotorPower());
            telemetry.addData("Hood Position", robot.shooter.getServoPosition());
            telemetry.update();
        }
    }
}
