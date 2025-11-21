package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Manual Control OpMode without Launch Sequence
 * 
 * This OpMode provides pure manual control of all robot subsystems without
 * any automated launch sequences. Perfect for testing, practice, and situations
 * where you want direct control over every aspect of the robot.
 * 
 * Controls:
 * - Left Stick: Forward/backward and strafe
 * - Right Stick: Rotation
 * - All subsystem controls handled through Robot.manualUpdate()
 */
@TeleOp(name = "MC No Launch Sequence", group = "Game")
public class ManualControlOpMode extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the robot in legacy mode (no enhanced aiming)
        robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Manual Control Initialized");
        telemetry.addData("Mode", "Pure Manual - No Launch Sequence");
        telemetry.addData("Info", "Press START to begin driving");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            robot.update(gamepad1);
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
    }
    
    /**
     * Displays sensor readings and limit switch states.
     */
    private void displaySensorTelemetry() {
        telemetry.addData("=== SENSORS ===", "");
        telemetry.addData("Intake Left Distance (cm)", "%.2f", robot.intake.getLeftDistance(DistanceUnit.CM));
        telemetry.addData("Intake Right Distance (cm)", "%.2f", robot.intake.getRightDistance(DistanceUnit.CM));
        telemetry.addData("Object Detected", robot.intake.isObjectDetected() ? "YES" : "NO");
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
        telemetry.addData("Limelight Target", robot.limelight.hasTarget() ? robot.limelight.getFiducialId() : "None");
    }
}
