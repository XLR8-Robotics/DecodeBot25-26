package org.firstinspires.ftc.teamcode.TeleOps;

import com.bylazar.field.FieldManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "TeleOpWithAutoAiming", group = "Game")
public class TeleOpWithAutoAim extends LinearOpMode {
    private Robot robot;
    private FieldManager panelsField;
    @Override
    public void runOpMode() throws InterruptedException {

        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        robot = new Robot(hardwareMap, follower);

        while (!isStarted() && !isStopRequested()) {
            follower.update();
        }
        
        waitForStart();

        while (opModeIsActive()) {
            robot.UpdateGamePad1(gamepad1);
            robot.UpdateGamePad2AutoAim(gamepad2);
            robot.RunAutoAim();
            displayTelemetry();
        }
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
        telemetry.addData("Shooter Stopper Position", "%.2f", robot.turret.getShooterBlockerPosition());
        telemetry.addData("Shooter States", robot.shooter.getCurrentState());
    }
    private void displaySensorTelemetry() {
        telemetry.addData("=== SENSORS ===", "");
        telemetry.addData("Turret Left Limit", robot.turret.isLeftLimitPressed() ? "PRESSED" : "Open");
        telemetry.addData("Turret Right Limit", robot.turret.isRightLimitPressed() ? "PRESSED" : "Open");
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

