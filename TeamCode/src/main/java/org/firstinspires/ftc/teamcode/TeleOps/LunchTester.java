package org.firstinspires.ftc.teamcode.TeleOps;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "Lunch thing", group = "Game")
public class LunchTester extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.turret.setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION);

        telemetry.addData("Status", "Manual Control Initialized");
        telemetry.addData("Mode", "Pure Manual - No Launch Sequence");
        telemetry.addData("Info", "Press START to begin driving");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            robot.launchSequenceController.update(gamepad1);

            // Launch Sequence Telemetry
            telemetry.addData("Launch Sequence Status", robot.launchSequenceController.getStatusString());

            // Shooter Telemetry
            telemetry.addData("Shooter Power", robot.shooter.getMotorPower());
            telemetry.addData("Shooter Hood Position", robot.shooter.getServoPosition());
            telemetry.addData("Shooter RPM", robot.shooter.getCurrentRPM());
            telemetry.addData("Shooter Status", robot.shooter.getDetailedStatusString());


            // Intake Telemetry
            telemetry.addData("Intake Power", robot.intake.getMotorPower());
            telemetry.addData("Intake Lift Position", robot.intake.getLiftServoPosition());

            telemetry.update();
        }

    }

}
