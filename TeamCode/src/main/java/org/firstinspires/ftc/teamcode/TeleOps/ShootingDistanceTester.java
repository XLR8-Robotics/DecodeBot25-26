package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@TeleOp(name = "Shooting Distance Tester / PID Tuner", group = "TeleOp")
public class ShootingDistanceTester extends LinearOpMode {

    private Robot robot;
    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the robot
        robot = new Robot(hardwareMap);
        
        // Initialize Dashboard Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Manual Control Initialized");
        telemetry.addData("Controls", "GP1: Drive/Intake | GP2: Turret/Shooter");
        telemetry.addData("Tuning Tip", "Keep I & D at 0. Tune F first, then P for recovery.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Apply PIDF coefficients
            robot.shooter.setPIDFCoefficients(p, i, d, f);

            robot.UpdateGamePad1(gamepad1);
            robot.UpdateGamePad2(gamepad2);
            
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
        displayPIDTelemetry();
        telemetry.update();
    }
    
    private void displayPIDTelemetry() {
        telemetry.addData("=== PID TUNING ===", "");
        telemetry.addData("P", p);
        telemetry.addData("I", i);
        telemetry.addData("D", d);
        telemetry.addData("F", f);
        telemetry.addData("Target RPM", robot.shooter.getCurrentState().rpm);
        telemetry.addData("Actual RPM", robot.shooter.getCurrentRPM());
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
        if (robot.shooter.isShooterMotorDisabled()) {
             telemetry.addData("Shooter Status", "DISABLED (Press Triangle to Enable)");
        } else {
             telemetry.addData("Shooter Status", "ENABLED");
        }
    }
}
