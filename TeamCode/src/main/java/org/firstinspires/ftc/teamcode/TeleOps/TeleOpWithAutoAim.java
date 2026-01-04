package org.firstinspires.ftc.teamcode.TeleOps;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Configurable
@TeleOp(name = "TeleOpWithAutoAiming", group = "Game")
public class TeleOpWithAutoAim extends OpMode {

    public static double P = 0.05;
    public static double I = 0.01;
    public static double D = 0.01;
    public static double F = 0.5;
    private Robot robot;
    private Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap, follower);

        robot.autoAimingTurret.setPIDFValues(P, I, D, F);
        robot.autoAimingTurret.setTowerPosition(new Pose(159.59, 130.03));

        robot.shooter.setRPM(0);
        robot.autoAimingTurret.limelightHardware.start();

        follower.setStartingPose(new Pose(110, -24, Math.toRadians(90)));
        follower.update();

        telemetry.addLine("TeleOpWithAutoAim Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        resetRuntime();
    }

    @Override
    public void loop() {
        robot.shooter.setRPM(0);
        // Drive & mechanisms
        robot.autoAimingTurret.setPIDFValues(P, I, D, F);
        robot.UpdateGamePad1(gamepad1);
        robot.UpdateGamePad2AutoAim(gamepad2);

        // Update robot pose
        follower.update();

        robot.autoAimingTurret.update();

        displayTelemetry();
    }

    private void displayTelemetry() {
        telemetry.addData("Shooter Power", "%.2f", robot.shooter.getMotorPower());
        telemetry.addData("Hood Position", "%.2f", robot.shooter.getServoPosition());
        telemetry.addData("Lift Servo Position", "%.2f", robot.intake.getLiftServoPosition());
        telemetry.addData("Shooter State", robot.shooter.getCurrentState());
        telemetry.addData("follower Heading", robot.autoAimingTurret.follower.getHeading());
        telemetry.addData("follower x", robot.autoAimingTurret.follower.getPose().getX());
        telemetry.addData("follower Y", robot.autoAimingTurret.follower.getPose().getY());
        telemetry.addData("lime light Pose x", robot.autoAimingTurret.limeLightPositon.getX());
        telemetry.addData("lime light pose y", robot.autoAimingTurret.limeLightPositon.getY());
        telemetry.addData("TargetTicks", robot.autoAimingTurret.turretTargetTicks);

        telemetry.addData("Turret Field Angle (deg)",
                robot.autoAimingTurret.targetFieldDeg);

        telemetry.update();
    }
}
