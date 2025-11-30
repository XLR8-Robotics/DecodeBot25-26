package org.firstinspires.ftc.teamcode.TeleOps;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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
    private TelemetryManager telemetryM;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        robot = new Robot(hardwareMap);
        robot.turret.setLimelight(robot.limelight);
        robot.limelight.start();
        robot.shooter.setRPM(0);
        
        try {
            follower = org.firstinspires.ftc.teamcode.pedropathing.Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(0, 0, 0));
            follower.startTeleopDrive();
            telemetryM.debug("Follower: Initialized");
        } catch (Exception e) {
            telemetryM.debug("Warning: Follower init failed: " + e.getMessage());
        }
        
        // Alliance Selection
        while (!isStarted() && !isStopRequested()) {
            robot.shooter.setRPM(0);
            if (gamepad1.square) {
                robot.setTargetSide(Robot.TargetSide.BLUE);
            } else if (gamepad1.circle) {
                robot.setTargetSide(Robot.TargetSide.RED);
            }
            
            telemetryM.debug("Status: Initialized");
            telemetryM.debug("ALLIANCE: " + robot.getTargetSide());
            int id = (robot.getTargetSide() == Robot.TargetSide.RED) ? FieldConstants.RED_APRILTAG_ID : FieldConstants.BLUE_APRILTAG_ID;
            telemetryM.debug("Target ID: " + id);
            telemetryM.debug("Select Alliance: Press X for BLUE, B for RED");
            telemetryM.update(telemetry);
        }
        
        // Configure Turret Target based on selection
        int targetId = (robot.getTargetSide() == Robot.TargetSide.RED) 
            ? FieldConstants.RED_APRILTAG_ID 
            : FieldConstants.BLUE_APRILTAG_ID;
        robot.turret.setTargetFiducialId(targetId);

        robot.turret.storeTargetPosition(20, 0);
        
        waitForStart();

        while (opModeIsActive()) {

            robot.limelight.updateResult();
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
                robot.turret.simpleAutoTrack();
            }

            displayTelemetry(robotHeadingDegrees);
        }
        robot.limelight.stop();
    }
    private void displayTelemetry(double heading) {
        telemetryM.debug("=== AUTO-AIM STATUS ===");
        telemetryM.debug("Enabled: " + isAutoAimEnabled);
        telemetryM.debug("State: " + robot.turret.getState());
        telemetryM.debug("Tracking: " + (robot.turret.isTracking() ? "LOCKED" : "SEARCHING/LOST"));
        
        telemetryM.debug("=== ODOMETRY ===");
        telemetryM.debug(String.format("Robot Heading: %.1f°", heading));
        if (follower != null) {
             telemetryM.debug(String.format("Pose X: %.1f", follower.getPose().getX()));
             telemetryM.debug(String.format("Pose Y: %.1f", follower.getPose().getY()));
        }
        
        telemetryM.debug("=== TURRET ===");
        telemetryM.debug(String.format("Angle: %.2f°", robot.turret.getAngle()));
        if (robot.turret.isTracking()) {
            telemetryM.debug(String.format("Target Dist: %.1f in", robot.turret.getLastKnownDistance()));
        }
        
        telemetryM.debug("=== LIMELIGHT ===");
        telemetryM.debug("Has Target: " + robot.limelight.hasTargets());
        if (robot.limelight.hasTargets()) {
            telemetryM.debug("Tag ID: " + robot.limelight.getBestTagId());
            telemetryM.debug(String.format("TX: %.2f", robot.limelight.getTx()));
        }
        
        telemetryM.update(telemetry);
    }
}
