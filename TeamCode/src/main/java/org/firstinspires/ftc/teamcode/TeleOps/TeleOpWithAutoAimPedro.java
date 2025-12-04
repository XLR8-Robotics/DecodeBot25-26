package org.firstinspires.ftc.teamcode.TeleOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "TeleOpWithAutoAimingPedro", group = "Game")
public class TeleOpWithAutoAimPedro extends OpMode {

    private Robot robot;
    private Follower follower;

    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    @Override
    public void init() {
        // Initialize follower and set starting pose
        follower = Constants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap, follower);

        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        follower.update();

        telemetry.addLine("TeleOpWithAutoAimPedro Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        // Start Pedro Pathing teleop drive
        follower.startTeleopDrive();
        resetRuntime();
    }

    @Override
    public void loop() {
        // Update turret, shooter, and intake
        robot.UpdateGamePad2AutoAim(gamepad2);

        // Run auto-aim logic
        robot.RunAutoAim();

        // Update Pedro Pathing follower
        follower.update();

        // TeleOp drive inputs
        double driveY = -gamepad1.left_stick_y;  // forward/back
        double driveX = -gamepad1.left_stick_x;  // left/right
        double turn = -gamepad1.right_stick_x;   // rotation

        // Slow mode toggle (right bumper)
        slowMode = gamepad1.right_bumper;

        if (slowMode) {
            driveY *= slowModeMultiplier;
            driveX *= slowModeMultiplier;
            turn *= slowModeMultiplier;
        }

        // Field-centric drive (true = robot-centric, false = field-centric)
        follower.setTeleOpDrive(driveY, driveX, turn, false);

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
        telemetry.addData("Shooter Stopper Position", "%.2f", robot.autoAimingTurret.getShooterBlockerPosition());
        telemetry.addData("Shooter States", robot.shooter.getCurrentState());
    }

    private void displaySensorTelemetry() {
        telemetry.addData("=== SENSORS ===", "");
        telemetry.addData("Turret Left Limit", robot.autoAimingTurret.isLeftLimitPressed() ? "PRESSED" : "Open");
        telemetry.addData("Turret Right Limit", robot.autoAimingTurret.isRightLimitPressed() ? "PRESSED" : "Open");
    }

    private void displayAutoAimTelemetry() {
        telemetry.addData("=== Auto Aiming Turret Statuses ===", "");
        telemetry.addData("Target Visible", robot.autoAimingTurret.getHasValidTarget());
        telemetry.addData("Status", robot.autoAimingTurret.getTurretStatus());

        telemetry.addData("=== Auto Aiming Turret Data ===", "");
        telemetry.addData("Robot Heading", "%.2f", robot.autoAimingTurret.getCurrentRobotHeading());
        telemetry.addData("Current Turret Angle", "%.2f", robot.autoAimingTurret.getCurrentTurretAngle());
        telemetry.addData("Last Field Angle", "%.2f", robot.autoAimingTurret.getLastKnownTargetAngleField());
        telemetry.addData("Desired Turret Angle", "%.2f", robot.autoAimingTurret.getDesiredTurretAngle());
        telemetry.addData("Target Lost Timer", "%.2f", robot.autoAimingTurret.getTargetLostTimer());
    }
}
