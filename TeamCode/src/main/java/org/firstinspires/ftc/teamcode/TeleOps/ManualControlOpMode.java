package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Manual Control OpMode
 * 
 * This OpMode provides manual control of all robot subsystems.
 * 
 * Controls:
 * - Gamepad 1: Drivetrain and Intake
 * - Gamepad 2: Turret, Launch Sequence, and Shooter
 */
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
            robot.UpdateGamePad2NoLS(gamepad2);
            displayTelemetry();
        }
    }

    /**
     * Displays comprehensive telemetry data organized by category.
     */
    private void displayTelemetry() {
        displayMotorAndServoTelemetry();
        displaySensorTelemetry();
        displaySystemStatusTelemetry();
        telemetry.update();
    }
    
    /**
     * Displays motor power and servo position telemetry.
     */
    private void displayMotorAndServoTelemetry() {
        telemetry.addData("=== MOTORS & SERVOS ===", "");
        telemetry.addData("Turret Power", "%.2f", robot.turret.getMotorPower());
        telemetry.addData("Intake Power", "%.2f", robot.intake.getMotorPower());
        telemetry.addData("Shooter Power", "%.2f", robot.shooter.getMotorPower());
        telemetry.addData("Hood Position", "%.2f", robot.shooter.getServoPosition());
        telemetry.addData("Lift Servo Position", "%.2f", robot.intake.getLiftServoPosition());
        telemetry.addData("Shooter Stopper Position", "%.2f", robot.turret.getShooterBlockerPosition());
        telemetry.addData("Shooter States", robot.shooter.getCurrentState());
    }
    
    /**
     * Displays sensor readings and limit switch states.
     */
    private void displaySensorTelemetry() {
        telemetry.addData("=== SENSORS ===", "");
       /* telemetry.addData("Intake Left Distance (cm)", "%.2f", robot.intake.getLeftDistance(DistanceUnit.CM));
        telemetry.addData("Intake Right Distance (cm)", "%.2f", robot.intake.getRightDistance(DistanceUnit.CM));
        telemetry.addData("Object Detected", robot.intake.isObjectDetected() ? "YES" : "NO");*/
        telemetry.addData("Turret Left Limit", robot.turret.isLeftLimitPressed() ? "PRESSED" : "Open");
        telemetry.addData("Turret Right Limit", robot.turret.isRightLimitPressed() ? "PRESSED" : "Open");
        telemetry.addData("Turret Angle", "%.2f degrees", robot.turret.getAngle());
    }
    
    /**
     * Displays system status including launch sequence and targeting info.
     */
    private void displaySystemStatusTelemetry() {
        telemetry.addData("=== SYSTEM STATUS ===", "");
        telemetry.addData("Launch Sequence", robot.getLaunchSequenceState());
        telemetry.addData("Target Side", robot.getTargetSide().toString());
        if (robot.shooter.isShooterMotorDisabled()) {
             telemetry.addData("Shooter Status", "DISABLED (Press Triangle to Enable)");
        } else {
             telemetry.addData("Shooter Status", "ENABLED");
        }
    }
}
