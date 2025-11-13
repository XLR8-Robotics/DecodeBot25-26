package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.constants.MecanumConstants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Advanced Autonomous Program - Competitive Strategy!
 * 
 * This autonomous demonstrates advanced concepts for competitive play:
 * - Multiple scoring cycles
 * - Adaptive strategy based on time remaining
 * - Error recovery and fallback plans
 * - Optimized path planning
 * - Performance tracking and optimization
 * 
 * Strategy:
 * 1. Preload shot (score immediate piece)
 * 2. First cycle: collect 2 pieces, shoot both
 * 3. Second cycle: collect more pieces if time allows
 * 4. Endgame: strategic parking for bonus points
 */
@Autonomous(name = "Advanced Auto Program", group = "Autos", preselectTeleOp = "MC No Launch Sequence")
public class AdvancedAutoProgram extends LinearOpMode {
    
    // =================================================================================
    // CONFIGURATION
    // =================================================================================
    
    // Alliance selection (change this based on your alliance)
    private final String ALLIANCE = "BLUE"; // or "RED"
    private final String STARTING_POSITION = "RIGHT"; // or "LEFT"
    
    // Timing configuration
    private final double MAX_AUTO_TIME = 30.0; // FTC autonomous period
    private final double PARKING_TIME_BUFFER = 3.0; // Reserve time for parking
    
    // =================================================================================
    // SUBSYSTEM INSTANCES
    // =================================================================================
    
    private Robot robot;
    private Follower follower;
    private ElapsedTime autoTimer = new ElapsedTime();
    
    // =================================================================================
    // STRATEGY STATE MACHINE
    // =================================================================================
    
    private enum StrategyState {
        PRELOAD_SHOT("Scoring preloaded piece"),
        FIRST_CYCLE_COLLECT("Collecting pieces - Cycle 1"),
        FIRST_CYCLE_SCORE("Scoring pieces - Cycle 1"),
        SECOND_CYCLE_COLLECT("Collecting pieces - Cycle 2"),
        SECOND_CYCLE_SCORE("Scoring pieces - Cycle 2"),
        ENDGAME_PARK("Strategic parking"),
        EMERGENCY_PARK("Emergency parking"),
        COMPLETE("Strategy complete");
        
        private final String description;
        
        StrategyState(String description) {
            this.description = description;
        }
        
        public String getDescription() { return description; }
    }
    
    private StrategyState currentState = StrategyState.PRELOAD_SHOT;
    private ElapsedTime stateTimer = new ElapsedTime();
    
    // =================================================================================
    // PERFORMANCE TRACKING
    // =================================================================================
    
    private int totalShotsTaken = 0;
    private int totalPiecesCollected = 0;
    private int successfulCycles = 0;
    private double averageCycleTime = 0.0;
    
    // =================================================================================
    // MAIN AUTONOMOUS METHODS
    // =================================================================================
    
    @Override
    public void runOpMode() {
        initializeRobot();
        
        displayStrategyInfo();
        waitForStart();
        
        if (isStopRequested()) return;
        
        autoTimer.reset();
        stateTimer.reset();
        
        // Main strategy execution loop
        while (opModeIsActive() && currentState != StrategyState.COMPLETE) {
            updateRobotSystems();
            executeCurrentStrategy();
            
            // Adaptive time management - switch to parking if running low on time
            if (AutoUtils.getRemainingTime(autoTimer, MAX_AUTO_TIME) < PARKING_TIME_BUFFER && 
                currentState != StrategyState.ENDGAME_PARK && 
                currentState != StrategyState.EMERGENCY_PARK) {
                transitionToState(StrategyState.EMERGENCY_PARK);
            }
            
            displayAdvancedTelemetry();
        }
        
        // Final performance report
        displayFinalReport();
        
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
        // Get starting position based on alliance and strategy
        Pose startPose = AutoUtils.getStartingPosition(ALLIANCE, STARTING_POSITION);
        
        // Initialize pedropathing
        follower = new Follower(hardwareMap, MecanumConstants.class);
        follower.setStartingPose(startPose);
        
        // Initialize robot with all advanced systems
        robot = new Robot(follower);
        robot.init(hardwareMap, Robot.AimingMode.ENHANCED, Robot.ShootingMode.AUTOMATIC);
        
        // Set alliance target
        robot.setTargetSide(ALLIANCE.equalsIgnoreCase("BLUE") ? Robot.TargetSide.BLUE : Robot.TargetSide.RED);
    }
    
    private void displayStrategyInfo() {
        telemetry.addData("Auto", "ADVANCED AUTONOMOUS");
        telemetry.addLine();
        telemetry.addData("Alliance", ALLIANCE);
        telemetry.addData("Starting Position", STARTING_POSITION);
        telemetry.addLine();
        telemetry.addData("STRATEGY:", "");
        telemetry.addData("1", "Preload shot (immediate score)");
        telemetry.addData("2", "Cycle 1: Collect & score 2 pieces");
        telemetry.addData("3", "Cycle 2: Additional pieces if time");
        telemetry.addData("4", "Strategic endgame parking");
        telemetry.addLine();
        telemetry.addData("Features", "Adaptive timing");
        telemetry.addData("", "Error recovery");
        telemetry.addData("", "Performance optimization");
        telemetry.addData("", "Multi-cycle scoring");
        telemetry.update();
    }
    
    // =================================================================================
    // STRATEGY EXECUTION
    // =================================================================================
    
    private void executeCurrentStrategy() {
        switch (currentState) {
            case PRELOAD_SHOT:
                executePreloadShot();
                break;
            case FIRST_CYCLE_COLLECT:
                executeFirstCycleCollect();
                break;
            case FIRST_CYCLE_SCORE:
                executeFirstCycleScore();
                break;
            case SECOND_CYCLE_COLLECT:
                executeSecondCycleCollect();
                break;
            case SECOND_CYCLE_SCORE:
                executeSecondCycleScore();
                break;
            case ENDGAME_PARK:
                executeEndgamePark();
                break;
            case EMERGENCY_PARK:
                executeEmergencyPark();
                break;
            case COMPLETE:
                // Strategy complete - hold position
                break;
        }
    }
    
    private void executePreloadShot() {
        // Quick shot with preloaded piece to get immediate points
        Pose shootingPos = AutoUtils.getShootingPosition(ALLIANCE);
        
        // Navigate to optimal shooting position
        if (follower.getPose().getDistanceToPoint(shootingPos.toPoint()) > 12.0) {
            AutoUtils.navigateToPosition(follower, shootingPos, 0.7, 8.0, 3.0);
        }
        
        // Take the preload shot using automated systems
        boolean shotTaken = AutoUtils.aimAndShoot(robot, 4.0, 3.0);
        if (shotTaken) {
            totalShotsTaken++;
            // Wait for launch sequence to complete
            AutoUtils.waitForLaunchComplete(robot, 3.0);
        }
        
        transitionToState(StrategyState.FIRST_CYCLE_COLLECT);
    }
    
    private void executeFirstCycleCollect() {
        // Collect multiple pieces efficiently
        Pose collectPos = ALLIANCE.equalsIgnoreCase("BLUE") ? 
            AutoUtils.FieldPositions.BLUE_ALLIANCE_PIECES : 
            AutoUtils.FieldPositions.RED_ALLIANCE_PIECES;
        
        // Navigate to collection area while running intake
        int piecesFound = AutoUtils.collectWhileNavigating(robot, follower, collectPos, 0.6, 8.0, 4.0);
        totalPiecesCollected += piecesFound;
        
        // Additional stationary collection if needed
        if (piecesFound < 2 && stateTimer.seconds() < 6.0) {
            boolean foundMore = AutoUtils.runIntakeUntilDetected(robot, 2.0, true);
            if (foundMore) totalPiecesCollected++;
        }
        
        transitionToState(StrategyState.FIRST_CYCLE_SCORE);
    }
    
    private void executeFirstCycleScore() {
        // Move to scoring position and shoot all collected pieces
        Pose shootingPos = AutoUtils.getShootingPosition(ALLIANCE);
        
        // Navigate to shooting position
        AutoUtils.navigateToPosition(follower, shootingPos, 0.8, 10.0, 3.0);
        
        // Take up to 2 shots using automated systems
        for (int shot = 0; shot < 2 && stateTimer.seconds() < 10.0; shot++) {
            boolean shotTaken = AutoUtils.aimAndShoot(robot, 3.0, 3.0);
            if (shotTaken) {
                totalShotsTaken++;
                AutoUtils.waitForLaunchComplete(robot, 2.0);
            }
            
            // Brief pause between shots if taking multiple
            if (shot < 1) {
                try { Thread.sleep(500); } catch (InterruptedException e) { break; }
            }
        }
        
        successfulCycles++;
        
        // Decide on next action based on remaining time
        if (AutoUtils.hasTimeFor(autoTimer, 8.0, MAX_AUTO_TIME, PARKING_TIME_BUFFER)) {
            transitionToState(StrategyState.SECOND_CYCLE_COLLECT);
        } else {
            transitionToState(StrategyState.ENDGAME_PARK);
        }
    }
    
    private void executeSecondCycleCollect() {
        // More aggressive collection for second cycle - try neutral pieces
        Pose neutralCollectPos = AutoUtils.FieldPositions.NEUTRAL_PIECES_CENTER;
        int piecesFound = AutoUtils.collectWhileNavigating(robot, follower, neutralCollectPos, 0.8, 6.0, 3.0);
        totalPiecesCollected += piecesFound;
        
        if (piecesFound > 0) {
            transitionToState(StrategyState.SECOND_CYCLE_SCORE);
        } else {
            // No pieces found, go straight to parking
            transitionToState(StrategyState.ENDGAME_PARK);
        }
    }
    
    private void executeSecondCycleScore() {
        // Quick scoring for second cycle using automated systems
        Pose shootingPos = AutoUtils.getShootingPosition(ALLIANCE);
        
        // Quick navigation to shooting position if not already close
        if (follower.getPose().getDistanceToPoint(shootingPos.toPoint()) > 15.0) {
            AutoUtils.navigateToPosition(follower, shootingPos, 0.9, 12.0, 2.0);
        }
        
        boolean shotTaken = AutoUtils.aimAndShoot(robot, 2.5, 4.0);
        if (shotTaken) {
            totalShotsTaken++;
            AutoUtils.waitForLaunchComplete(robot, 1.5);
            successfulCycles++;
        }
        
        transitionToState(StrategyState.ENDGAME_PARK);
    }
    
    private void executeEndgamePark() {
        // Strategic parking for maximum points
        Pose parkPos = AutoUtils.getParkingPosition(ALLIANCE);
        
        // Navigate to parking area
        AutoUtils.navigateToPosition(follower, parkPos, 0.7, 8.0, 2.5);
        
        transitionToState(StrategyState.COMPLETE);
    }
    
    private void executeEmergencyPark() {
        // Quick parking when time is running out - use nearest safe position
        Pose emergencyParkPos = new Pose(follower.getPose().getX() + 24, follower.getPose().getY(), follower.getPose().getHeading());
        AutoUtils.navigateToPosition(follower, emergencyParkPos, 0.9, 6.0, 1.5);
        transitionToState(StrategyState.COMPLETE);
    }
    
    // =================================================================================
    // HELPER METHODS
    // =================================================================================
    
    private void updateRobotSystems() {
        follower.update();
        robot.limelight.update();
        
        if (robot.getAimingController() != null) {
            robot.getAimingController().update();
        }
        
        if (robot.getShootingController() != null) {
            robot.getShootingController().update();
        }
    }
    
    private void transitionToState(StrategyState newState) {
        // Calculate cycle time for performance tracking
        if (currentState == StrategyState.FIRST_CYCLE_SCORE || currentState == StrategyState.SECOND_CYCLE_SCORE) {
            double cycleTime = stateTimer.seconds();
            averageCycleTime = (averageCycleTime * (successfulCycles - 1) + cycleTime) / successfulCycles;
        }
        
        currentState = newState;
        stateTimer.reset();
    }
    
    // =================================================================================
    // TELEMETRY METHODS
    // =================================================================================
    
    private void displayAdvancedTelemetry() {
        telemetry.clear();
        telemetry.addData("Auto", "ADVANCED AUTONOMOUS");
        telemetry.addData("Strategy State", currentState.getDescription());
        telemetry.addData("State Time", "%.1fs", stateTimer.seconds());
        telemetry.addData("Total Time", "%.1fs / %.0fs", autoTimer.seconds(), MAX_AUTO_TIME);
        
        // Progress indicator
        double progress = autoTimer.seconds() / MAX_AUTO_TIME;
        String progressBar = "#".repeat((int)(progress * 15)) + "-".repeat(15 - (int)(progress * 15));
        telemetry.addData("Progress", progressBar + String.format(" %.0f%%", progress * 100));
        
        telemetry.addLine();
        
        // Performance metrics
        telemetry.addData("=== PERFORMANCE ===", "");
        telemetry.addData("Shots Taken", totalShotsTaken);
        telemetry.addData("Pieces Collected", totalPiecesCollected);
        telemetry.addData("Cycles Completed", successfulCycles);
        if (averageCycleTime > 0) {
            telemetry.addData("Avg Cycle Time", "%.1fs", averageCycleTime);
        }
        
        telemetry.addLine();
        
        // Current position and target info
        Pose currentPose = follower.getPose();
        telemetry.addData("Position", "X:%.0f Y:%.0f H:%.0f°", 
            currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        
        if (robot.limelight.hasTarget()) {
            telemetry.addData("Target", "ID:" + robot.limelight.getFiducialId());
        } else {
            telemetry.addData("Target", "Searching...");
        }
        
        // Time management
        double timeRemaining = AutoUtils.getRemainingTime(autoTimer, MAX_AUTO_TIME);
        telemetry.addData("Time Remaining", "%.1fs", timeRemaining);
        
        if (timeRemaining < PARKING_TIME_BUFFER) {
            telemetry.addData("WARNING", "Low time - prioritizing parking!");
        }
        
        telemetry.update();
    }
    
    private void displayFinalReport() {
        telemetry.clear();
        telemetry.addData("Auto", "AUTONOMOUS COMPLETE!");
        telemetry.addLine();
        
        telemetry.addData("FINAL PERFORMANCE:", "");
        telemetry.addData("Total Time Used", "%.1f / %.0f seconds", autoTimer.seconds(), MAX_AUTO_TIME);
        telemetry.addData("Shots Taken", totalShotsTaken);
        telemetry.addData("Pieces Collected", totalPiecesCollected);
        telemetry.addData("Cycles Completed", successfulCycles);
        
        // Calculate efficiency metrics
        double timeEfficiency = (autoTimer.seconds() / MAX_AUTO_TIME) * 100;
        double shotRate = totalShotsTaken / autoTimer.seconds() * 60; // shots per minute
        
        telemetry.addLine();
        telemetry.addData("Time Efficiency", "%.0f%%", timeEfficiency);
        telemetry.addData("Shot Rate", "%.1f shots/minute", shotRate);
        
        if (successfulCycles >= 2) {
            telemetry.addData("Strategy Result", "EXCELLENT!");
        } else if (successfulCycles >= 1) {
            telemetry.addData("Strategy Result", "Good execution");
        } else {
            telemetry.addData("Strategy Result", "Needs improvement");
        }
        
        telemetry.update();
    }
}
