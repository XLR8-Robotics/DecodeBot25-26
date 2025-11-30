package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.teamcode.config.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "Auto Aim TeleOp", group = "Game")
public class AutoAimTeleOp extends LinearOpMode {

    private Robot robot;
    private Follower follower;
    private boolean previousTriangleButtonState = false;
    private boolean isAutoAimEnabled = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.turret.setLimelight(robot.limelight);
        robot.shooter.setRPM(0);
        
        try {
            follower = org.firstinspires.ftc.teamcode.pedropathing.Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(0, 0, 0));
            follower.startTeleopDrive();
            telemetry.addData("Follower", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Warning", "Follower init failed: " + e.getMessage());
        }
        
        // Alliance Selection
        while (!isStarted() && !isStopRequested()) {
            robot.shooter.setRPM(0);
            if (gamepad1.square) {
                robot.setTargetSide(Robot.TargetSide.BLUE);
            } else if (gamepad1.circle) {
                robot.setTargetSide(Robot.TargetSide.RED);
            }
            
            telemetry.addData("Status", "Initialized");
            telemetry.addData("ALLIANCE", robot.getTargetSide());
            int id = (robot.getTargetSide() == Robot.TargetSide.RED) ? FieldConstants.RED_APRILTAG_ID : FieldConstants.BLUE_APRILTAG_ID;
            telemetry.addData("Target ID", id);
            telemetry.addData("Select Alliance", "Press X for BLUE, B for RED");
            telemetry.update();
        }
        
        // Configure Turret Target based on selection
        int targetId = (robot.getTargetSide() == Robot.TargetSide.RED) 
            ? FieldConstants.RED_APRILTAG_ID 
            : FieldConstants.BLUE_APRILTAG_ID;
        robot.turret.setTargetFiducialId(targetId);
        
        waitForStart();

        while (opModeIsActive()) {

            robot.UpdateGamePad1(gamepad1);
            robot.UpdateGamePad2(gamepad2);
            
            double robotHeadingDegrees = 0.0;
            if (follower != null) {
                follower.update();
                
                if (gamepad1.start || gamepad1.options) {
                    follower.setPose(new Pose(0, 0, 0));
                }

                robotHeadingDegrees = Math.toDegrees(follower.getPose().getHeading());
            }

            boolean currentTriangleButtonState = gamepad1.triangle;
            if (currentTriangleButtonState && !previousTriangleButtonState) {
                isAutoAimEnabled = !isAutoAimEnabled;
                robot.turret.enableAutoAim(isAutoAimEnabled);
            }
            previousTriangleButtonState = currentTriangleButtonState;
            
            if (isAutoAimEnabled) {
                robot.turret.update(robotHeadingDegrees);
            }

            displayTelemetry(robotHeadingDegrees);
        }
    }
    private void displayTelemetry(double heading) {
        telemetry.addData("=== AUTO-AIM STATUS ===", "");
        telemetry.addData("Enabled", isAutoAimEnabled);
        telemetry.addData("State", robot.turret.getState());
        telemetry.addData("Tracking", robot.turret.isTracking() ? "LOCKED" : "SEARCHING/LOST");
        
        telemetry.addData("=== ODOMETRY ===", "");
        telemetry.addData("Robot Heading", "%.1f°", heading);
        if (follower != null) {
             telemetry.addData("Pose X", "%.1f", follower.getPose().getX());
             telemetry.addData("Pose Y", "%.1f", follower.getPose().getY());
        }
        
        telemetry.addData("=== TURRET ===", "");
        telemetry.addData("Angle", "%.2f°", robot.turret.getAngle());
        if (robot.turret.isTracking()) {
            telemetry.addData("Target Dist", "%.1f in", robot.turret.getLastKnownDistance());
        }
        
        telemetry.addData("=== LIMELIGHT ===", "");
        telemetry.addData("Has Target", robot.limelight.hasTarget());
        if (robot.limelight.hasTarget()) {
            telemetry.addData("Tag ID", robot.limelight.getFiducialId());
            telemetry.addData("TX", "%.2f", robot.limelight.getTx());
        }
        
        telemetry.update();
    }
}
