package org.firstinspires.ftc.teamcode.TeleOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "TeleOpWithAutoAiming", group = "Game")
public class TeleOpWithAutoAim extends OpMode {

    private Robot robot;
    private Follower follower;

    @Override
    public void init() {
        // Initialize follower and set starting pose
        follower = Constants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap, follower);

        // Set robot starting pose (x, y, heading in radians)
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        follower.update();

        telemetry.addLine("TeleOpWithAutoAim Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        resetRuntime();
    }

    @Override
    public void loop() {
        // Update gamepads
        robot.UpdateGamePad1(gamepad1);             // Drive + intake
        robot.UpdateGamePad2AutoAim(gamepad2);     // Shooter + turret manual
        // Run auto-aim logic
        robot.runAutoAim(gamepad2);

        // Update follower heading
        follower.update();

        // Update telemetry
        displayTelemetry();
    }

    private void displayTelemetry() {
        displayAutoAimTelemetry();
        displayMotorAndServoTelemetry();
        displaySensorTelemetry();
        telemetry.update();
    }

    private void displayMotorAndServoTelemetry() {
        telemetry.addData("=== MOTORS & SERVOS ===", "");
        telemetry.addData("Shooter Power", "%.2f", robot.shooter.getMotorPower());
        telemetry.addData("Hood Position", "%.2f", robot.shooter.getServoPosition());
        telemetry.addData("Lift Servo Position", "%.2f", robot.intake.getLiftServoPosition());
        telemetry.addData("Shooter States", robot.shooter.getCurrentState());
    }

    private void displaySensorTelemetry() {
        telemetry.addData("=== SENSORS ===", "");
        telemetry.addData("Turret Left Limit", robot.autoAimingTurret.isLeftLimitPressed() ? "PRESSED" : "Open");
        telemetry.addData("Turret Right Limit", robot.autoAimingTurret.isRightLimitPressed() ? "PRESSED" : "Open");
    }

    private void displayAutoAimTelemetry() {
        telemetry.addData("=== Auto Aiming Turret Statuses ===", "");

    }
}
