package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "MCNoLaunceSequence", group = "Game")
public class ManualControlOpMode extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the robot
        robot = new Robot();
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized and Ready to Drive!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // The main robot update loop handles all logic
            robot.manualUpdate(gamepad1);

            // --- Telemetry ---
            displayTelemetry();
        }
    }

    private void displayTelemetry() {
        // Display the power of all motors and position of servos
        telemetry.addData("Turret Power", "%.2f", robot.turret.getMotorPower());
        telemetry.addData("Intake Power", "%.2f", robot.intake.getMotorPower());
        telemetry.addData("Shooter Power", "%.2f", robot.shooter.getMotorPower());
        telemetry.addData("Hood Position", "%.2f", robot.shooter.getServoPosition());
        telemetry.addData("Lift Servo Position", "%.2f", robot.intake.getLiftServoPosition());
        telemetry.addData("Shooter Stopper Position", "%.2f", robot.turret.getShooterBlockerPosition());

        // --- Sensor Data ---
        telemetry.addData("Intake Left Distance (cm)", "%.2f", robot.intake.getLeftDistance(DistanceUnit.CM));
        telemetry.addData("Intake Right Distance (cm)", "%.2f", robot.intake.getRightDistance(DistanceUnit.CM));
        telemetry.addData("Object Detected", robot.intake.isObjectDetected());
        telemetry.addData("Turret Left Limit", robot.turret.isLeftLimitPressed());
        telemetry.addData("Turret Right Limit", robot.turret.isRightLimitPressed());
        telemetry.addData("Turret Angle", "%.2f degrees", robot.turret.getAngle());

        // --- Launch Sequence State ---
        telemetry.addData("Launch Sequence", robot.getLaunchSequenceState());

        // --- Turret Targeting Info ---
        telemetry.addData("Target Side", robot.getTargetSide().toString());
        telemetry.addData("Target Tag ID", robot.getCurrentTargetTagId());
        telemetry.addData("Limelight Target", robot.limelight.hasTarget() ? robot.limelight.getFiducialId() : "None");

        telemetry.update();
    }
}
