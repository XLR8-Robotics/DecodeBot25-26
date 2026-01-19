package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name =
                "░█▀▀▄░░░░░░░░░░░▄▀▀█\n" +
                "░█░░░▀▄░▄▄▄▄▄░▄▀░░░█\n" +
                "░░▀▄░░░▀░░░░░▀░░░▄▀\n" +
                "░░░░▌░▄▄░░░▄▄░▐▀▀\n" +
                "░░░▐░░█▄░░░▄█░░▌▄▄▀▀▀▀█\n" +
                "░░░▌▄▄▀▀░▄░▀▀▄▄▐░░░░░░█\n" +
                "▄▀▀▐▀▀░▄▄▄▄▄░▀▀▌▄▄▄░░░█\n" +
                "█░░░▀▄░█░░░█░▄▀░░░░█▀▀▀\n" +
                "░▀▄░░▀░░▀▀▀░░▀░░░▄█▀\n" +
                "░░░█░░░░░░░░░░░▄▀▄░▀▄\n" +
                "░░░█░░░░░░░░░▄▀█░░█░░█\n" +
                "░░░█░░░░░░░░░░░█▄█░░▄▀\n" +
                "░░░█░░░░░░░░░░░████▀\n" +
                "░░░▀▄▄▀▀▄▄▀▀▄▄▄█▀", group = "Game")
public class MotorDisabledManualControlOpMode extends OpMode {

    private Robot robot;
    private boolean hasRumbled = false;
    private ElapsedTime timer;

    @Override
    public void init() {
        // Initialize the robot
        robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Manual Control Initialized");
        telemetry.addData("Controls", "GP1: Drive/Intake | GP2: Turret/Shooter");
        telemetry.update();

        robot.shooter.setRPM(0);
        timer = new ElapsedTime();
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {

        robot.UpdateGamePad1(gamepad1);
        robot.UpdateGamePad2(gamepad2);
        notifyLast15Seconds(timer);
        displayTelemetry();
        robot.shooter.setRPM(0);
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

    private void notifyLast15Seconds(ElapsedTime timer) {
        if (timer.seconds() >= 101 && !hasRumbled) {
            gamepad1.rumble(500);
            gamepad2.rumble(500);
            hasRumbled = true;
        }
    }
}
