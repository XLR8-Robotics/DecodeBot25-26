package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.constants.MecanumConstants;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.teamcode.config.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Simple Autonomous Program - Perfect for Learning!
 * 
 * This is a straightforward autonomous that demonstrates basic concepts:
 * 1. Drive forward to intake area
 * 2. Run intake for a few seconds
 * 3. Drive to shooting position
 * 4. Shoot using launch sequence
 * 5. Park
 * 
 * Great starting point for teams new to autonomous programming!
 */
@Autonomous(name = "Simple Auto Program", group = "Autos", preselectTeleOp = "MC No Launch Sequence")
public class SimpleAutoProgram extends LinearOpMode {
    
    // =================================================================================
    // ROBOT SYSTEMS
    // =================================================================================
    
    private Robot robot;
    private Follower follower;
    private ElapsedTime autoTimer = new ElapsedTime();
    
    // =================================================================================
    // FIELD POSITIONS (adjust these for your field setup)
    // =================================================================================
    
    // Starting position
    private final Pose startPose = new Pose(0, -60, Math.toRadians(90));
    
    // Intake position
    private final Pose intakePosition = new Pose(-24, -36, Math.toRadians(0));
    
    // Shooting position
    private final Pose shootingPosition = new Pose(36, 36, Math.toRadians(135));
    
    // Parking position
    private final Pose parkPosition = new Pose(60, -60, Math.toRadians(0));
    
    @Override
    public void runOpMode() {
        // Initialize robot
        initializeRobot();
        
        // Show autonomous plan
        displayAutoPlan();
        
        waitForStart();
        
        if (isStopRequested()) return;
        
        autoTimer.reset();
        
        // === AUTONOMOUS SEQUENCE ===
        
        // Step 1: Drive forward to intake area (2 seconds)
        driveForwardToIntake();
        
        // Step 2: Collect game pieces (3 seconds)
        collectGamePieces();
        
        // Step 3: Drive to shooting position (3 seconds)
        moveToShootingPosition();
        
        // Step 4: Aim and shoot (5 seconds)
        aimAndShoot();
        
        // Step 5: Park (2 seconds)
        parkRobot();
        
        // Final cleanup
        robot.stopAllMotors();
        
        telemetry.addData("STATUS", "AUTONOMOUS COMPLETE!");
        telemetry.addData("Total Time", "%.1f seconds", autoTimer.seconds());
        telemetry.update();
        
        // Hold position
        while (opModeIsActive()) {
            follower.update();
            sleep(20);
        }
    }
    
    // =================================================================================
    // INITIALIZATION
    // =================================================================================
    
    private void initializeRobot() {
        // Initialize pedropathing
        follower = new Follower(hardwareMap, MecanumConstants.class);
        follower.setStartingPose(startPose);
        
        // Initialize robot with all systems
        robot = new Robot(follower);
        robot.init(hardwareMap, Robot.AimingMode.ENHANCED, Robot.ShootingMode.AUTOMATIC);
        
        // Set target (change to RED if needed)
        robot.setTargetSide(Robot.TargetSide.BLUE);
    }
    
    private void displayAutoPlan() {
        telemetry.addData("Auto", "SIMPLE AUTONOMOUS");
        telemetry.addLine();
        telemetry.addData("PLAN:", "");
        telemetry.addData("Step 1", "Drive forward (2s)");
        telemetry.addData("Step 2", "Collect pieces (3s)");
        telemetry.addData("Step 3", "Move to shoot (3s)");
        telemetry.addData("Step 4", "Aim & shoot (5s)");
        telemetry.addData("Step 5", "Park (2s)");
        telemetry.addLine();
        telemetry.addData("Total Time", "~15 seconds");
        telemetry.addData("Alliance", "BLUE (change in code for RED)");
        telemetry.update();
    }
    
    // =================================================================================
    // AUTONOMOUS STEPS
    // =================================================================================
    
    private void driveForwardToIntake() {
        telemetry.addData("STATUS", "STEP 1: Navigating to intake area...");
        telemetry.update();
        
        ElapsedTime stepTimer = new ElapsedTime();
        
        // Create and follow path to intake position
        follower.setMaxPower(0.7);
        follower.followPath(follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(startPose.getX(), startPose.getY(), Point.CARTESIAN),
                new Point(intakePosition.getX(), intakePosition.getY(), Point.CARTESIAN)
            ))
            .setLinearHeadingInterpolation(startPose.getHeading(), intakePosition.getHeading())
            .build());
        
        // Wait until we reach the intake position or timeout
        while (opModeIsActive() && stepTimer.seconds() < 5.0) {
            follower.update();
            
            double distanceToTarget = follower.getPose().getDistanceToPoint(intakePosition.toPoint());
            displayStepTelemetry("Navigating to Intake", stepTimer, 5.0);
            
            telemetry.addData("Distance to Target", "%.1f inches", distanceToTarget);
            telemetry.update();
            
            // Break if we're close to the target
            if (distanceToTarget < 6.0) {
                break;
            }
        }
    }
    
    private void collectGamePieces() {
        telemetry.addData("STATUS", "STEP 2: Collecting game pieces...");
        telemetry.update();
        
        ElapsedTime stepTimer = new ElapsedTime();
        
        // Run intake and lift
        robot.intake.setPower(Constants.IntakeConfig.INTAKE_SPEED);
        robot.intake.setLiftPosition(Constants.IntakeConfig.LIFT_SERVO_LIFTING_POSITION);
        
        while (opModeIsActive() && stepTimer.seconds() < 3.0) {
            follower.update(); // Keep updating position
            
            displayStepTelemetry("Collecting Pieces", stepTimer, 3.0);
            
            // Show if we detected something
            if (robot.intake.isObjectDetected()) {
                telemetry.addData("Status", "Game piece detected!");
            } else {
                telemetry.addData("Status", "Searching...");
            }
            telemetry.update();
        }
        
        // Stop intake and lower lift
        robot.intake.setPower(0);
        robot.intake.setLiftPosition(Constants.IntakeConfig.LIFT_SERVO_NOT_LIFTING_POSITION);
    }
    
    private void moveToShootingPosition() {
        telemetry.addData("STATUS", "STEP 3: Navigating to shooting position...");
        telemetry.update();
        
        ElapsedTime stepTimer = new ElapsedTime();
        
        // Create and follow path to shooting position
        follower.setMaxPower(0.8);
        follower.followPath(follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                new Point(shootingPosition.getX(), shootingPosition.getY(), Point.CARTESIAN)
            ))
            .setLinearHeadingInterpolation(follower.getPose().getHeading(), shootingPosition.getHeading())
            .build());
        
        // Wait until we reach the shooting position or timeout
        while (opModeIsActive() && stepTimer.seconds() < 6.0) {
            follower.update();
            
            double distanceToTarget = follower.getPose().getDistanceToPoint(shootingPosition.toPoint());
            displayStepTelemetry("Navigating to Shoot", stepTimer, 6.0);
            
            telemetry.addData("Distance to Target", "%.1f inches", distanceToTarget);
            telemetry.update();
            
            // Break if we're close to the target
            if (distanceToTarget < 8.0) {
                break;
            }
        }
    }
    
    private void aimAndShoot() {
        telemetry.addData("STATUS", "STEP 4: Auto-aiming and launch sequence...");
        telemetry.update();
        
        ElapsedTime stepTimer = new ElapsedTime();
        boolean launchSequenceStarted = false;
        boolean launchComplete = false;
        
        // Start auto-aiming at target
        if (robot.getAimingController() != null) {
            int targetId = (robot.getTargetSide() == Robot.TargetSide.RED) 
                ? FieldConstants.RED_APRILTAG_ID 
                : FieldConstants.BLUE_APRILTAG_ID;
            robot.getAimingController().setTargetAprilTagId(targetId);
            
            // Set turret to aiming mode
            robot.turret.setState(org.firstinspires.ftc.teamcode.subsystems.Turret.TurretState.AIMING);
        }
        
        // Enable automatic shooting for distance-based control
        if (robot.getShootingController() != null) {
            robot.getShootingController().setAutoShootingEnabled(true);
        }
        
        while (opModeIsActive() && stepTimer.seconds() < 8.0 && !launchComplete) {
            follower.update();
            robot.limelight.update();
            
            // Update aiming system
            if (robot.getAimingController() != null) {
                robot.getAimingController().update();
            }
            
            // Update shooting system for distance-based parameters
            if (robot.getShootingController() != null) {
                robot.getShootingController().update();
            }
            
            // Update launch sequence controller
            robot.launchSequenceController.update(null); // No gamepad in auto
            
            displayStepTelemetry("Auto-Aiming & Launch", stepTimer, 8.0);
            
            // Show targeting info
            if (robot.limelight.hasTarget()) {
                double aimError = Math.abs(robot.limelight.getTx());
                telemetry.addData("Target", "AprilTag ID: " + robot.limelight.getFiducialId());
                telemetry.addData("Aim Error", "%.1f degrees", aimError);
                telemetry.addData("Distance", "%.1f inches", robot.limelight.getDistanceToTarget());
                
                // Start launch sequence when aimed (within 3 degrees and after 1.5 seconds)
                if (!launchSequenceStarted && stepTimer.seconds() > 1.5 && aimError < 3.0) {
                    boolean sequenceStarted = robot.startLaunchSequence();
                    if (sequenceStarted) {
                        launchSequenceStarted = true;
                        telemetry.addData("Launch", "LAUNCH SEQUENCE STARTED!");
                    }
                }
            } else {
                telemetry.addData("Target", "Searching for AprilTag...");
            }
            
            // Check launch sequence status
            if (launchSequenceStarted) {
                if (robot.launchSequenceController.isRunning()) {
                    telemetry.addData("Launch Status", robot.launchSequenceController.getCurrentState());
                } else {
                    telemetry.addData("Launch Status", "COMPLETE!");
                    launchComplete = true;
                }
            } else {
                telemetry.addData("Launch Status", "Aiming...");
            }
            
            telemetry.update();
        }
        
        // Stop automated systems
        if (robot.getAimingController() != null) {
            robot.getAimingController().stopAiming();
            robot.turret.setState(org.firstinspires.ftc.teamcode.subsystems.Turret.TurretState.MANUAL);
        }
        
        if (robot.getShootingController() != null) {
            robot.getShootingController().setAutoShootingEnabled(false);
        }
    }
    
    private void parkRobot() {
        telemetry.addData("STATUS", "STEP 5: Navigating to parking zone...");
        telemetry.update();
        
        ElapsedTime stepTimer = new ElapsedTime();
        
        // Navigate to parking position
        follower.setMaxPower(0.6);
        follower.followPath(follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                new Point(parkPosition.getX(), parkPosition.getY(), Point.CARTESIAN)
            ))
            .setLinearHeadingInterpolation(follower.getPose().getHeading(), parkPosition.getHeading())
            .build());
        
        // Wait until we reach the parking position or timeout
        while (opModeIsActive() && stepTimer.seconds() < 4.0) {
            follower.update();
            
            double distanceToTarget = follower.getPose().getDistanceToPoint(parkPosition.toPoint());
            displayStepTelemetry("Parking", stepTimer, 4.0);
            
            telemetry.addData("Distance to Target", "%.1f inches", distanceToTarget);
            telemetry.update();
            
            // Break if we're close to the target
            if (distanceToTarget < 8.0) {
                break;
            }
        }
    }
    
    // =================================================================================
    // HELPER METHODS
    // =================================================================================
    
    private void displayStepTelemetry(String stepName, ElapsedTime stepTimer, double maxTime) {
        telemetry.clear();
        telemetry.addData("Auto", "SIMPLE AUTONOMOUS");
        telemetry.addData("Current Step", stepName);
        telemetry.addData("Step Progress", "%.1f / %.1f seconds", stepTimer.seconds(), maxTime);
        
        // Progress bar
        double progress = Math.min(1.0, stepTimer.seconds() / maxTime);
        int progressBars = (int)(progress * 10);
        StringBuilder progressBar = new StringBuilder();
        for (int i = 0; i < 10; i++) {
            progressBar.append(i < progressBars ? "#" : "-");
        }
        telemetry.addData("Progress", progressBar.toString() + String.format(" %.0f%%", progress * 100));
        
        telemetry.addLine();
        telemetry.addData("Total Auto Time", "%.1f seconds", autoTimer.seconds());
        
        // Robot position
        Pose currentPose = follower.getPose();
        telemetry.addData("Position", "X:%.0f Y:%.0f H:%.0f°", 
            currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
    }
}
