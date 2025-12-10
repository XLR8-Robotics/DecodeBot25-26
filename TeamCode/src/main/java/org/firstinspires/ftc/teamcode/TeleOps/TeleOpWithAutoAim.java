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
        robot.autoAimingTurret.setTarget(159.23, 129.69);
        robot.shooter.setRPM(0);

        // Set robot starting pose (x, y, heading in radians) — back wall at y = 0
        follower.setStartingPose(new Pose(72, 0, Math.toRadians(90)));
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

    }

    private void displayAutoAimTelemetry() {
        telemetry.addData("=== Auto Aiming Turret Statuses ===", "");
        Pose pose = follower.getPose();
        double headingDeg = Math.toDegrees(follower.getHeading());
        double turretAngle = robot.autoAimingTurret.getAngle();

        double targetX = robot.autoAimingTurret.getTargetX();
        double targetY = robot.autoAimingTurret.getTargetY();

        double fieldAngle = Math.toDegrees(Math.atan2(
                targetY - pose.getY(),
                targetX - pose.getX()
        ));

        double turretTarget = ((fieldAngle - headingDeg + 180) % 360 + 360) % 360;

        telemetry.addData("Bot Pose (in)", "X: %.1f  Y: %.1f  Heading: %.1f deg", pose.getX(), pose.getY(), headingDeg);
        telemetry.addData("Turret", "Angle: %.1f deg  TargetAngle: %.1f deg", turretAngle, turretTarget);
        telemetry.addData("Aim Target (in)", "X: %.1f  Y: %.1f", targetX, targetY);
    }
}
