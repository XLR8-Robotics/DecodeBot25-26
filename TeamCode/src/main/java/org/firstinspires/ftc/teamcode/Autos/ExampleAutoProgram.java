package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.Vector;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.localization.constants.MecanumConstants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.teamcode.config.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Example Autonomous Program for DECODE Season
 * 
 * This autonomous demonstrates a complete game sequence:
 * 1. Start from starting position
 * 2. Move to intake area and collect game pieces
 * 3. Navigate to shooting position
 * 4. Aim at AprilTag and shoot collected pieces
 * 5. Optional second cycle for more pieces
 * 
 * Features Demonstrated:
 * - Pedropathing autonomous navigation
 * - Automated intake control
 * - Automatic turret aiming system
 * - Distance-based shooting system
 * - Launch sequence automation
 * - State machine autonomous structure
 */
@Autonomous(name = "Example Auto Program", group = "Autos", preselectTeleOp = "MC No Launch Sequence")
public class ExampleAutoProgram extends LinearOpMode {
    
    // =================================================================================
    // SUBSYSTEM INSTANCES
    // =================================================================================
    
    private Robot robot;
    private Follower follower;
    
    // =================================================================================
    // AUTONOMOUS STATE MACHINE
    // =================================================================================
    
    private enum AutoState {
        START("Moving to intake position"),
        INTAKE_APPROACH("Approaching intake area"),
        COLLECTING("Collecting game pieces"),
        MOVE_TO_SHOOT("Moving to shooting position"),
        AIM_AND_SHOOT("Aiming and shooting"),
        CYCLE_BACK("Returning for more pieces"),
        PARK("Parking in end zone"),
        COMPLETE("Autonomous complete");
        
        private final String description;
        
        AutoState(String description) {
            this.description = description;
        }
        
        public String getDescription() {
            return description;
        }
    }
    
    private AutoState currentState = AutoState.START;
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime autoTimer = new ElapsedTime();
    
    // =================================================================================
    // FIELD POSITIONS (inches from field center)
    // =================================================================================
    
    // Starting position (adjust based on your alliance and starting tile)
    private final Pose startPose = new Pose(12, -60, Math.toRadians(90));
    
    // Intake area (where game pieces are located)
    private final Pose intakePosition = new Pose(-24, -36, Math.toRadians(0));
    
    // Shooting position (optimal distance from AprilTag)
    private final Pose shootingPosition = new Pose(36, 36, Math.toRadians(135));
    
    // Parking position
    private final Pose parkPosition = new Pose(60, -60, Math.toRadians(0));
    
    // =================================================================================
    // AUTONOMOUS PATHS
    // =================================================================================
    
    private PathChain startToIntakePath;
    private PathChain intakeToShootPath;
    private PathChain shootToParkPath;
    
    // =================================================================================
    // AUTONOMOUS CONTROL VARIABLES
    // =================================================================================
    
    private boolean piecesCollected = false;
    private boolean aimingComplete = false;
    private boolean shootingComplete = false;
    private int shotsFired = 0;
    private final int maxShots = 3; // Maximum shots to take
    
    // =================================================================================
    // MAIN AUTONOMOUS METHODS
    // =================================================================================
    
    @Override
    public void runOpMode() {
        initializeRobot();
        buildPaths();
        
        // Display autonomous information
        displayAutoInfo();
        
        waitForStart();
        
        if (isStopRequested()) return;
        
        autoTimer.reset();
        stateTimer.reset();
        
        // Main autonomous loop
        while (opModeIsActive() && currentState != AutoState.COMPLETE) {
            // Update robot systems
            updateRobotSystems();
            
            // Execute current state
            executeCurrentState();
            
            // Display autonomous telemetry
            displayAutoTelemetry();
            
            // Safety timeout (30 seconds)
            if (autoTimer.seconds() > 30.0) {
                transitionToState(AutoState.COMPLETE);
            }
        }
        
        // Final cleanup
        robot.stopAllMotors();
    }
    
    // =================================================================================
    // INITIALIZATION METHODS
    // =================================================================================
    
    private void initializeRobot() {
        // Initialize pedropathing follower
        follower = new Follower(hardwareMap, MecanumConstants.class);
        follower.setStartingPose(startPose);
        
        // Initialize robot with all advanced systems
        robot = new Robot(follower);
        robot.init(hardwareMap, Robot.AimingMode.ENHANCED, Robot.ShootingMode.AUTOMATIC);
        
        // Set target to Blue AprilTag (change to RED if needed)
        robot.setTargetSide(Robot.TargetSide.BLUE);
        
        // Enable automatic shooting system for distance-based control
        if (robot.getShootingController() != null) {
            robot.getShootingController().setAutoShootingEnabled(true);
        }
    }
    
    private void buildPaths() {
        // Path 1: Start to intake area
        startToIntakePath = follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(startPose.getX(), startPose.getY(), Point.CARTESIAN),
                new Point(intakePosition.getX(), intakePosition.getY(), Point.CARTESIAN)
            ))
            .setLinearHeadingInterpolation(startPose.getHeading(), intakePosition.getHeading())
            .build();
        
        // Path 2: Intake area to shooting position
        intakeToShootPath = follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(intakePosition.getX(), intakePosition.getY(), Point.CARTESIAN),
                new Point(0, 0, Point.CARTESIAN), // Control point for smooth curve
                new Point(shootingPosition.getX(), shootingPosition.getY(), Point.CARTESIAN)
            ))
            .setLinearHeadingInterpolation(intakePosition.getHeading(), shootingPosition.getHeading())
            .build();
        
        // Path 3: Shooting position to park
        shootToParkPath = follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(shootingPosition.getX(), shootingPosition.getY(), Point.CARTESIAN),
                new Point(parkPosition.getX(), parkPosition.getY(), Point.CARTESIAN)
            ))
            .setLinearHeadingInterpolation(shootingPosition.getHeading(), parkPosition.getHeading())
            .build();
    }
    
    private void displayAutoInfo() {
        telemetry.addData("Auto", "EXAMPLE AUTONOMOUS PROGRAM");
        telemetry.addLine();
        telemetry.addData("Alliance", "BLUE (Change in code if RED)");
        telemetry.addData("Strategy", "Intake -> Move -> Shoot -> Park");
        telemetry.addData("Features", "All automated systems active");
        telemetry.addLine();
        telemetry.addData("Starting Position", "X:%.0f Y:%.0f H:%.0f°", 
            startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        telemetry.addData("Estimated Time", "15-25 seconds");
        telemetry.addLine();
        telemetry.addData("Systems Active", "Pedropathing Navigation");
        telemetry.addData("", "Automated Intake");
        telemetry.addData("", "Turret Auto-Aiming");
        telemetry.addData("", "Distance-Based Shooting");
        telemetry.addData("", "Launch Sequence");
        telemetry.update();
    }
    
    // =================================================================================
    // STATE MACHINE IMPLEMENTATION
    // =================================================================================
    
    private void executeCurrentState() {
        switch (currentState) {
            case START:
                executeStartState();
                break;
            case INTAKE_APPROACH:
                executeIntakeApproachState();
                break;
            case COLLECTING:
                executeCollectingState();
                break;
            case MOVE_TO_SHOOT:
                executeMoveToShootState();
                break;
            case AIM_AND_SHOOT:
                executeAimAndShootState();
                break;
            case CYCLE_BACK:
                executeCycleBackState();
                break;
            case PARK:
                executeParkState();
                break;
            case COMPLETE:
                // Do nothing - autonomous is complete
                break;
        }
    }
    
    private void executeStartState() {
        // Start following path to intake area
        follower.followPath(startToIntakePath);
        transitionToState(AutoState.INTAKE_APPROACH);
    }
    
    private void executeIntakeApproachState() {
        // Wait until we're close to the intake position
        if (follower.getPose().getDistanceToPoint(intakePosition.toPoint()) < 6.0) {
            transitionToState(AutoState.COLLECTING);
        }
        
        // Timeout after 5 seconds
        if (stateTimer.seconds() > 5.0) {
            transitionToState(AutoState.COLLECTING);
        }
    }
    
    private void executeCollectingState() {
        // Run intake to collect game pieces
        robot.intake.setPower(Constants.IntakeConfig.INTAKE_SPEED);
        robot.intake.setLiftPosition(Constants.IntakeConfig.LIFT_SERVO_LIFTING_POSITION);
        
        // Check if we've collected pieces (using distance sensor)
        if (robot.intake.isObjectDetected()) {
            piecesCollected = true;
        }
        
        // Move on after collecting or timeout
        if (piecesCollected || stateTimer.seconds() > 3.0) {
            robot.intake.setPower(0);
            robot.intake.setLiftPosition(Constants.IntakeConfig.LIFT_SERVO_NOT_LIFTING_POSITION);
            
            // Start path to shooting position
            follower.followPath(intakeToShootPath);
            transitionToState(AutoState.MOVE_TO_SHOOT);
        }
    }
    
    private void executeMoveToShootState() {
        // Wait until we're close to the shooting position
        if (follower.getPose().getDistanceToPoint(shootingPosition.toPoint()) < 8.0) {
            // Start aiming at the target
            if (robot.getAimingController() != null) {
                int targetId = (robot.getTargetSide() == Robot.TargetSide.RED) 
                    ? FieldConstants.RED_APRILTAG_ID 
                    : FieldConstants.BLUE_APRILTAG_ID;
                robot.getAimingController().setTargetAprilTagId(targetId);
                
                // Set turret to aiming mode
                robot.turret.setState(org.firstinspires.ftc.teamcode.subsystems.Turret.TurretState.AIMING);
            }
            
            transitionToState(AutoState.AIM_AND_SHOOT);
        }
        
        // Timeout after 8 seconds
        if (stateTimer.seconds() > 8.0) {
            transitionToState(AutoState.AIM_AND_SHOOT);
        }
    }
    
    private void executeAimAndShootState() {
        // Check if we have a target and are aimed
        boolean hasTarget = robot.limelight.hasTarget();
        boolean isAimed = hasTarget && Math.abs(robot.limelight.getTx()) < 3.0; // Within 3 degrees
        
        if (hasTarget && isAimed && !aimingComplete) {
            aimingComplete = true;
            
            // Start launch sequence
            boolean sequenceStarted = robot.startLaunchSequence();
            if (sequenceStarted) {
                shotsFired++;
            }
        }
        
        // Check if launch sequence is complete
        if (aimingComplete && !robot.launchSequenceController.isRunning()) {
            shootingComplete = true;
        }
        
        // Move on after successful shot or timeout
        if (shootingComplete || stateTimer.seconds() > 10.0) {
            if (shotsFired < maxShots && piecesCollected && stateTimer.seconds() < 20.0) {
                // Go back for more pieces if time allows
                transitionToState(AutoState.CYCLE_BACK);
            } else {
                // Move to park
                follower.followPath(shootToParkPath);
                transitionToState(AutoState.PARK);
            }
        }
    }
    
    private void executeCycleBackState() {
        // This could implement a second cycle to collect more pieces
        // For now, just move to park after a brief delay
        if (stateTimer.seconds() > 2.0) {
            follower.followPath(shootToParkPath);
            transitionToState(AutoState.PARK);
        }
    }
    
    private void executeParkState() {
        // Wait until we're in the parking area
        if (follower.getPose().getDistanceToPoint(parkPosition.toPoint()) < 8.0 
            || stateTimer.seconds() > 5.0) {
            transitionToState(AutoState.COMPLETE);
        }
    }
    
    // =================================================================================
    // HELPER METHODS
    // =================================================================================
    
    private void updateRobotSystems() {
        // Update follower for path following
        follower.update();
        
        // Update robot systems (but not manual control since this is autonomous)
        robot.limelight.update();
        
        // Update aiming controller if active
        if (robot.getAimingController() != null) {
            robot.getAimingController().update();
        }
        
        // Update shooting controller if active
        if (robot.getShootingController() != null) {
            robot.getShootingController().update();
        }
        
        // Update launch sequence controller
        if (robot.launchSequenceController != null) {
            robot.launchSequenceController.update(null); // No gamepad in autonomous
        }
    }
    
    private void transitionToState(AutoState newState) {
        currentState = newState;
        stateTimer.reset();
        
        // Reset state-specific variables
        switch (newState) {
            case AIM_AND_SHOOT:
                aimingComplete = false;
                shootingComplete = false;
                break;
            // Add other state-specific resets as needed
        }
    }
    
    // =================================================================================
    // TELEMETRY METHODS
    // =================================================================================
    
    private void displayAutoTelemetry() {
        telemetry.addData("Auto", "AUTONOMOUS ACTIVE");
        telemetry.addData("State", currentState.getDescription());
        telemetry.addData("State Time", "%.1f seconds", stateTimer.seconds());
        telemetry.addData("Auto Time", "%.1f seconds", autoTimer.seconds());
        telemetry.addLine();
        
        displayPositionData();
        displaySystemStatus();
        displayPerformanceData();
        
        telemetry.update();
    }
    
    private void displayPositionData() {
        telemetry.addData("=== POSITION ===", "");
        Pose currentPose = follower.getPose();
        telemetry.addData("Robot Position", "X:%.1f Y:%.1f H:%.0f°", 
            currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        
        // Show distance to target based on current state
        switch (currentState) {
            case INTAKE_APPROACH:
                double distToIntake = currentPose.getDistanceToPoint(intakePosition.toPoint());
                telemetry.addData("Distance to Intake", "%.1f inches", distToIntake);
                break;
            case MOVE_TO_SHOOT:
                double distToShoot = currentPose.getDistanceToPoint(shootingPosition.toPoint());
                telemetry.addData("Distance to Shoot Pos", "%.1f inches", distToShoot);
                break;
            case PARK:
                double distToPark = currentPose.getDistanceToPoint(parkPosition.toPoint());
                telemetry.addData("Distance to Park", "%.1f inches", distToPark);
                break;
        }
        telemetry.addLine();
    }
    
    private void displaySystemStatus() {
        telemetry.addData("=== SYSTEMS ===", "");
        
        // Intake status
        telemetry.addData("Pieces Collected", piecesCollected ? "YES" : "NO");
        telemetry.addData("Object Detected", robot.intake.isObjectDetected() ? "YES" : "NO");
        
        // Shooting status
        if (robot.limelight.hasTarget()) {
            telemetry.addData("Target", "ID:" + robot.limelight.getFiducialId() + 
                " TX:" + String.format("%.1f°", robot.limelight.getTx()));
        } else {
            telemetry.addData("Target", "No AprilTag visible");
        }
        
        // Launch sequence status
        telemetry.addData("Launch Sequence", robot.launchSequenceController.getCurrentState().toString());
        telemetry.addData("Shots Fired", shotsFired + "/" + maxShots);
        telemetry.addLine();
    }
    
    private void displayPerformanceData() {
        telemetry.addData("=== PERFORMANCE ===", "");
        telemetry.addData("Path Following", follower.isBusy() ? "Active" : "Complete");
        telemetry.addData("Aiming Status", aimingComplete ? "Aimed" : "Aiming");
        telemetry.addData("Shooting Status", shootingComplete ? "Shot" : "Preparing");
        
        // Estimated completion time
        double estimatedTimeRemaining = Math.max(0, 25.0 - autoTimer.seconds());
        telemetry.addData("Est. Time Remaining", "%.0f seconds", estimatedTimeRemaining);
    }
}
