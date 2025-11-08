package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config // Make this class's static variables editable from the dashboard
@TeleOp(name = "Shooting Sequence Tuner", group = "Tuning")
public class ShootingSequenceTuner extends LinearOpMode {

    private Robot robot;
    private ElapsedTime timer = new ElapsedTime();

    // These are the values you will tune from the dashboard.
    public static double SHOOTER_POWER = 0.85;
    public static double LIFT_SERVO_UP_POSITION = 0.6;
    public static double LIFT_SERVO_DOWN_POSITION = 0.2;
    public static double SHOOTER_BLOCKER_OPEN_POSITION = 0.5;
    public static double SHOOTER_BLOCKER_CLOSED_POSITION = 0.9;
    public static double SPIN_UP_TIME_MS = 1000;
    public static double LIFT_TIME_MS = 500;
    public static double SHOOTER_BLOCKER_OPEN_TIME_MS = 500;

    private enum ShootingState {
        IDLE,
        SPINNING_UP,
        LIFTING,
        SHOOTING,
        RESETTING
    }

    private ShootingState currentShootingState = ShootingState.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        robot = new Robot();
        robot.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Ready for Shooting Sequence Tuning.");
        telemetry.addLine("Connect to http://192.168.43.1:8080/dash");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Shooting Sequence ---
            switch (currentShootingState) {
                case IDLE:
                    if (gamepad1.square) {
                        currentShootingState = ShootingState.SPINNING_UP;
                        timer.reset();
                        robot.shooter.setPower(SHOOTER_POWER);
                    }
                    break;
                case SPINNING_UP:
                    if (timer.milliseconds() >= SPIN_UP_TIME_MS) {
                        currentShootingState = ShootingState.LIFTING;
                        timer.reset();
                        robot.intake.setLiftPosition(LIFT_SERVO_UP_POSITION);
                    }
                    break;
                case LIFTING:
                    if (timer.milliseconds() >= LIFT_TIME_MS) {
                        currentShootingState = ShootingState.SHOOTING;
                        timer.reset();
                        robot.turret.setShooterBlockerPosition(SHOOTER_BLOCKER_OPEN_POSITION);
                    }
                    break;
                case SHOOTING:
                    if (timer.milliseconds() >= SHOOTER_BLOCKER_OPEN_TIME_MS) {
                        currentShootingState = ShootingState.RESETTING;
                        timer.reset();
                    }
                    break;
                case RESETTING:
                    robot.shooter.setPower(0);
                    robot.intake.setLiftPosition(LIFT_SERVO_DOWN_POSITION);
                    robot.turret.setShooterBlockerPosition(SHOOTER_BLOCKER_CLOSED_POSITION);
                    currentShootingState = ShootingState.IDLE;
                    break;
            }

            // --- Telemetry ---
            telemetry.addLine("--- Shooting Sequence Tuning ---");
            telemetry.addData("Shooting State", currentShootingState);
            telemetry.addLine();
            telemetry.addData("Shooter Power", SHOOTER_POWER);
            telemetry.addData("Lift Servo Up", LIFT_SERVO_UP_POSITION);
            telemetry.addData("Lift Servo Down", LIFT_SERVO_DOWN_POSITION);
            telemetry.addData("Shooter Blocker Open", SHOOTER_BLOCKER_OPEN_POSITION);
            telemetry.addData("Shooter Blocker Closed", SHOOTER_BLOCKER_CLOSED_POSITION);
            telemetry.addData("Spin Up Time (ms)", SPIN_UP_TIME_MS);
            telemetry.addData("Lift Time (ms)", LIFT_TIME_MS);
            telemetry.addData("Shooter Blocker Open Time (ms)", SHOOTER_BLOCKER_OPEN_TIME_MS);
            telemetry.addLine();
            telemetry.addData("Shooter Motor Power", robot.shooter.getMotorPower());

            telemetry.update();
        }
    }
}
