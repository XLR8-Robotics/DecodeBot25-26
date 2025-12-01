package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "Manual Tele Op", group = "Game")
public class ManualControlOpMode extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the robot
        robot = new Robot(hardwareMap);
        telemetry.addData("Status", "Manual Control Initialized");
        telemetry.addData("Controls", "GP1: Drive/Intake | GP2: Turret/Shooter");

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            robot.UpdateGamePad1(gamepad1);
            robot.UpdateGamePad2(gamepad2);
            displayTelemetry();
        }
    }

    private void displayTelemetry() {
        displayMotorAndServoTelemetry();
        displaySensorTelemetry();
        displaySystemStatusTelemetry();
        telemetry.update();
    }
    private void displayMotorAndServoTelemetry() {
        telemetry.addData("=== MOTORS & SERVOS ===", "");
        telemetry.addData("Shooter Power", "%.2f", robot.shooter.getMotorPower());
        telemetry.addData("Hood Position", "%.2f", robot.shooter.getServoPosition());
        telemetry.addData("Lift Servo Position", "%.2f", robot.intake.getLiftServoPosition());
        telemetry.addData("Shooter Stopper Position", "%.2f", robot.turret.getShooterBlockerPosition());
        telemetry.addData("Shooter States", robot.shooter.getCurrentState());
    }
    private void displaySensorTelemetry() {
        telemetry.addData("=== SENSORS ===", "");
        telemetry.addData("Turret Left Limit", robot.turret.isLeftLimitPressed() ? "PRESSED" : "Open");
        telemetry.addData("Turret Right Limit", robot.turret.isRightLimitPressed() ? "PRESSED" : "Open");
    }
    private void displaySystemStatusTelemetry() {
        telemetry.addData("=== SYSTEM STATUS ===", "");
        telemetry.addData("Target Side", robot.getTargetSide().toString());
        if (robot.shooter.isShooterMotorDisabled()) {
             telemetry.addData("Shooter Status", "DISABLED (Press Triangle to Enable)");
        } else {
             telemetry.addData("Shooter Status", "ENABLED");
        }
    }
}
