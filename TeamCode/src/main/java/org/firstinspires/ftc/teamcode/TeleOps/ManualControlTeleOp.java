package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "Manual Control", group = "TeleOp")
public class ManualControlTeleOp extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the robot, which in turn initializes all subsystems
        robot = new Robot();
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized and Ready to Drive!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Drivetrain Control ---
            // Get joystick values from gamepad 1
            double forward = -gamepad1.left_stick_y; // The y-axis is often reversed
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            robot.drivetrain.manualDrive(forward, strafe, turn);

            // --- Subsystem Control ---
            // Update all the other subsystems based on gamepad 1 input
            robot.turret.update(gamepad1);
            robot.intake.update(gamepad1);
            robot.shooter.update(gamepad1);

            // --- Telemetry ---
            // Display the power of all motors and position of servos
            double[] drivetrainPowers = robot.drivetrain.getMotorPowers();
            telemetry.addData("Left Front Power", "%.2f", drivetrainPowers[0]);
            telemetry.addData("Right Front Power", "%.2f", drivetrainPowers[1]);
            telemetry.addData("Left Rear Power", "%.2f", drivetrainPowers[2]);
            telemetry.addData("Right Rear Power", "%.2f", drivetrainPowers[3]);
            telemetry.addData("Turret Power", "%.2f", robot.turret.getMotorPower());
            telemetry.addData("Intake Power", "%.2f", robot.intake.getMotorPower());
            telemetry.addData("Shooter Power", "%.2f", robot.shooter.getMotorPower());
            telemetry.addData("Hood Position", "%.2f", robot.shooter.getServoPosition());
            telemetry.addData("Lift Servo Position", "%.2f", robot.intake.getLiftServoPosition());
            telemetry.addData("Shooter Stopper Position", "%.2f", robot.turret.getShooterBlockerPosition());
            telemetry.addData("Shooter is Running", robot.shooter.isRunning());
            telemetry.update();
        }
    }
}
