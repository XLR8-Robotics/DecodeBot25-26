package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "Shooter Control", group = "TeleOp")
public class ShooterTeleOp extends LinearOpMode {

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
            // Update the shooter based on gamepad 1 input
            robot.shooter.update(gamepad1);

            // Display the shooter motor power and hood servo position
            telemetry.addData("Shooter Power", robot.shooter.getMotorPower());
            telemetry.addData("Hood Position", robot.shooter.getServoPosition());
            telemetry.update();
        }
    }
}
