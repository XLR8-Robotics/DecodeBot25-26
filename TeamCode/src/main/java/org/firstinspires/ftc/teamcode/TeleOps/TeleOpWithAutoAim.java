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
    public static double D = 0.009;
    public static double F = 0.0;
    // Extra turret tuning values exposed for live adjustment (match AutoAimingTurret defaults)
    public static double MAX_POWER = 0.9;
    public static double STATIC_FF = 0.006;
    public static double ANGLE_DEADBAND_DEG = 0.75;
    public static double SMOOTH_MAX_DELTA_FAR = 0.18;
    public static double SMOOTH_MAX_DELTA_NEAR = 0.05;
    public static double LARGE_ERROR_DEG = 25.0;
    public static double TOWER_X = 130;
    public static double TOWER_Y = 130.03;
    private Robot robot;
    private Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap, follower);

        robot.shooter.setRPM(0);
        robot.autoAimingTurret.setPIDFValues(P, I, D, F);
        robot.autoAimingTurret.setMotionTuning(
                MAX_POWER, STATIC_FF, ANGLE_DEADBAND_DEG,
                SMOOTH_MAX_DELTA_FAR, SMOOTH_MAX_DELTA_NEAR, LARGE_ERROR_DEG);

        robot.autoAimingTurret.setTowerPosition(new Pose(TOWER_X, TOWER_Y));



        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
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
        robot.autoAimingTurret.setMotionTuning(
                MAX_POWER, STATIC_FF, ANGLE_DEADBAND_DEG,
                SMOOTH_MAX_DELTA_FAR, SMOOTH_MAX_DELTA_NEAR, LARGE_ERROR_DEG);
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
        telemetry.addData("follower Heading", Math.toDegrees(robot.autoAimingTurret.follower.getHeading()));
        telemetry.addData("follower x", robot.autoAimingTurret.follower.getPose().getX());
        telemetry.addData("follower Y", robot.autoAimingTurret.follower.getPose().getY());
        telemetry.addData("TargetTicks", robot.autoAimingTurret.turretTargetTicks);
        telemetry.addData("is shooter Blocked", robot.autoAimingTurret.isShooterBlocked);

        telemetry.addData("Turret Field Angle (deg)",
                robot.autoAimingTurret.targetFieldDeg);

        telemetry.update();
    }
}
