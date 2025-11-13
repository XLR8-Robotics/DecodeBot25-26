package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.teamcode.config.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Utility class containing common autonomous functions and helpers.
 * These methods can be used across multiple autonomous OpModes to reduce code duplication.
 */
public class AutoUtils {
    
    // =================================================================================
    // FIELD POSITIONS - DECODE Season 2025-2026
    // =================================================================================
    
    /**
     * Common field positions for DECODE season.
     * Adjust these based on actual field measurements and game strategy.
     */
    public static class FieldPositions {
        // Starting positions (adjust based on alliance color and starting tile choice)
        public static final Pose BLUE_START_LEFT = new Pose(-36, -60, Math.toRadians(90));
        public static final Pose BLUE_START_RIGHT = new Pose(12, -60, Math.toRadians(90));
        public static final Pose RED_START_LEFT = new Pose(-12, 60, Math.toRadians(270));
        public static final Pose RED_START_RIGHT = new Pose(36, 60, Math.toRadians(270));
        
        // Game piece locations (where balls/cargo are located)
        public static final Pose NEUTRAL_PIECES_CENTER = new Pose(0, 0, Math.toRadians(0));
        public static final Pose BLUE_ALLIANCE_PIECES = new Pose(-36, -24, Math.toRadians(180));
        public static final Pose RED_ALLIANCE_PIECES = new Pose(36, 24, Math.toRadians(0));
        
        // Optimal shooting positions (based on AprilTag locations)
        public static final Pose BLUE_SHOOTING_POSITION = new Pose(36, 36, Math.toRadians(135));
        public static final Pose RED_SHOOTING_POSITION = new Pose(-36, -36, Math.toRadians(315));
        
        // Parking/end game positions
        public static final Pose BLUE_PARK = new Pose(60, -60, Math.toRadians(0));
        public static final Pose RED_PARK = new Pose(-60, 60, Math.toRadians(180));
        public static final Pose NEUTRAL_PARK = new Pose(0, -72, Math.toRadians(90));
    }
    
    // =================================================================================
    // MOVEMENT UTILITIES
    // =================================================================================
    
    /**
     * Navigates to a target position using pedropathing.
     * @param follower The pedropathing follower
     * @param targetPose Target position and heading
     * @param maxPower Maximum power for movement (0.0 to 1.0)
     * @param tolerance Distance tolerance in inches
     * @param maxTimeSeconds Maximum time to attempt navigation
     * @return true if reached target, false if timed out
     */
    public static boolean navigateToPosition(Follower follower, Pose targetPose, double maxPower, double tolerance, double maxTimeSeconds) {
        ElapsedTime timer = new ElapsedTime();
        
        // Create and follow path to target
        follower.setMaxPower(maxPower);
        follower.followPath(follower.pathBuilder()
            .addPath(new com.pedropathing.pathgen.BezierLine(
                new com.pedropathing.pathgen.Point(follower.getPose().getX(), follower.getPose().getY(), com.pedropathing.pathgen.Point.CARTESIAN),
                new com.pedropathing.pathgen.Point(targetPose.getX(), targetPose.getY(), com.pedropathing.pathgen.Point.CARTESIAN)
            ))
            .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getHeading())
            .build());
        
        // Wait until we reach the target or timeout
        while (timer.seconds() < maxTimeSeconds) {
            follower.update();
            
            double distance = follower.getPose().getDistanceToPoint(targetPose.toPoint());
            if (distance <= tolerance) {
                return true;
            }
            
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                return false;
            }
        }
        
        return false; // Timed out
    }
    
    /**
     * Simple time-based movement for fine positioning or emergency situations.
     * Use navigateToPosition() for normal navigation.
     * @param follower The pedropathing follower
     * @param forward Forward speed (-1.0 to 1.0, negative is forward)
     * @param strafe Strafe speed (-1.0 to 1.0, positive is right)
     * @param turn Turn speed (-1.0 to 1.0, positive is clockwise)
     * @param timeSeconds Time to drive in seconds
     * @return true if completed successfully, false if interrupted
     */
    public static boolean driveForTime(Follower follower, double forward, double strafe, double turn, double timeSeconds) {
        ElapsedTime timer = new ElapsedTime();
        
        while (timer.seconds() < timeSeconds) {
            follower.setTeleOpMovementVectors(forward, strafe, turn);
            follower.update();
            
            // Small delay to prevent overwhelming the control loop
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                return false;
            }
        }
        
        // Stop movement
        follower.setTeleOpMovementVectors(0, 0, 0);
        follower.update();
        
        return true;
    }
    
    /**
     * Waits until the robot is within a certain distance of a target position.
     * @param follower The pedropathing follower
     * @param targetPose Target position
     * @param tolerance Distance tolerance in inches
     * @param maxTimeSeconds Maximum time to wait
     * @return true if reached target, false if timed out
     */
    public static boolean waitUntilNear(Follower follower, Pose targetPose, double tolerance, double maxTimeSeconds) {
        ElapsedTime timer = new ElapsedTime();
        
        while (timer.seconds() < maxTimeSeconds) {
            follower.update();
            
            double distance = follower.getPose().getDistanceToPoint(targetPose.toPoint());
            if (distance <= tolerance) {
                return true;
            }
            
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                return false;
            }
        }
        
        return false; // Timed out
    }
    
    // =================================================================================
    // INTAKE UTILITIES
    // =================================================================================
    
    /**
     * Runs the intake system for a specified time or until a piece is detected.
     * @param robot The robot instance
     * @param maxTimeSeconds Maximum time to run intake
     * @param waitForDetection If true, stops when piece is detected
     * @return true if piece was detected, false if timed out
     */
    public static boolean runIntakeUntilDetected(Robot robot, double maxTimeSeconds, boolean waitForDetection) {
        ElapsedTime timer = new ElapsedTime();
        boolean pieceDetected = false;
        
        // Start intake
        robot.intake.setPower(Constants.IntakeConfig.INTAKE_SPEED);
        robot.intake.setLiftPosition(Constants.IntakeConfig.LIFT_SERVO_LIFTING_POSITION);
        
        while (timer.seconds() < maxTimeSeconds) {
            if (robot.intake.isObjectDetected()) {
                pieceDetected = true;
                if (waitForDetection) {
                    break; // Stop immediately when detected
                }
            }
            
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                break;
            }
        }
        
        // Stop intake
        robot.intake.setPower(0);
        robot.intake.setLiftPosition(Constants.IntakeConfig.LIFT_SERVO_NOT_LIFTING_POSITION);
        
        return pieceDetected;
    }
    
    /**
     * Navigates to a collection area while running intake to collect pieces.
     * @param robot The robot instance
     * @param follower The pedropathing follower
     * @param targetPose Target position for collection
     * @param maxPower Maximum power for movement
     * @param tolerance Distance tolerance in inches
     * @param maxTimeSeconds Maximum time for collection
     * @return Number of pieces detected during collection
     */
    public static int collectWhileNavigating(Robot robot, Follower follower, 
                                          Pose targetPose, double maxPower, double tolerance, double maxTimeSeconds) {
        ElapsedTime timer = new ElapsedTime();
        int piecesCollected = 0;
        boolean previousDetection = false;
        
        // Start intake
        robot.intake.setPower(Constants.IntakeConfig.INTAKE_SPEED);
        robot.intake.setLiftPosition(Constants.IntakeConfig.LIFT_SERVO_LIFTING_POSITION);
        
        // Create and follow path to collection area
        follower.setMaxPower(maxPower);
        follower.followPath(follower.pathBuilder()
            .addPath(new com.pedropathing.pathgen.BezierLine(
                new com.pedropathing.pathgen.Point(follower.getPose().getX(), follower.getPose().getY(), com.pedropathing.pathgen.Point.CARTESIAN),
                new com.pedropathing.pathgen.Point(targetPose.getX(), targetPose.getY(), com.pedropathing.pathgen.Point.CARTESIAN)
            ))
            .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getHeading())
            .build());
        
        while (timer.seconds() < maxTimeSeconds) {
            follower.update();
            
            // Check for new piece detection
            boolean currentDetection = robot.intake.isObjectDetected();
            if (currentDetection && !previousDetection) {
                piecesCollected++;
            }
            previousDetection = currentDetection;
            
            // Break if we're close enough to the target
            double distance = follower.getPose().getDistanceToPoint(targetPose.toPoint());
            if (distance <= tolerance) {
                break;
            }
            
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                break;
            }
        }
        
        // Stop systems
        robot.intake.setPower(0);
        robot.intake.setLiftPosition(Constants.IntakeConfig.LIFT_SERVO_NOT_LIFTING_POSITION);
        
        return piecesCollected;
    }
    
    /**
     * Runs intake while driving with time-based movement (for fine collection).
     * @param robot The robot instance
     * @param follower The pedropathing follower
     * @param forward Forward speed
     * @param strafe Strafe speed
     * @param turn Turn speed
     * @param timeSeconds Time to drive with intake running
     * @return Number of pieces detected during collection
     */
    public static int collectWhileDriving(Robot robot, Follower follower, 
                                        double forward, double strafe, double turn, double timeSeconds) {
        ElapsedTime timer = new ElapsedTime();
        int piecesCollected = 0;
        boolean previousDetection = false;
        
        // Start intake
        robot.intake.setPower(Constants.IntakeConfig.INTAKE_SPEED);
        robot.intake.setLiftPosition(Constants.IntakeConfig.LIFT_SERVO_LIFTING_POSITION);
        
        while (timer.seconds() < timeSeconds) {
            // Drive
            follower.setTeleOpMovementVectors(forward, strafe, turn);
            follower.update();
            
            // Check for new piece detection
            boolean currentDetection = robot.intake.isObjectDetected();
            if (currentDetection && !previousDetection) {
                piecesCollected++;
            }
            previousDetection = currentDetection;
            
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                break;
            }
        }
        
        // Stop systems
        follower.setTeleOpMovementVectors(0, 0, 0);
        follower.update();
        robot.intake.setPower(0);
        robot.intake.setLiftPosition(Constants.IntakeConfig.LIFT_SERVO_NOT_LIFTING_POSITION);
        
        return piecesCollected;
    }
    
    // =================================================================================
    // SHOOTING UTILITIES
    // =================================================================================
    
    /**
     * Automatically aims at the appropriate AprilTag and executes launch sequence.
     * Uses the full automated aiming and shooting systems.
     * @param robot The robot instance
     * @param maxAimTime Maximum time to spend aiming in seconds
     * @param aimTolerance Aiming tolerance in degrees
     * @return true if shot was taken, false if failed to aim
     */
    public static boolean aimAndShoot(Robot robot, double maxAimTime, double aimTolerance) {
        ElapsedTime timer = new ElapsedTime();
        boolean launchSequenceStarted = false;
        boolean shotComplete = false;
        
        // Start auto-aiming at appropriate target
        if (robot.getAimingController() != null) {
            int targetId = (robot.getTargetSide() == Robot.TargetSide.RED) 
                ? FieldConstants.RED_APRILTAG_ID 
                : FieldConstants.BLUE_APRILTAG_ID;
            robot.getAimingController().setTargetAprilTagId(targetId);
            
            // Set turret to aiming mode
            robot.turret.setState(Turret.TurretState.AIMING);
        }
        
        // Enable automatic shooting for distance-based control
        if (robot.getShootingController() != null) {
            robot.getShootingController().setAutoShootingEnabled(true);
        }
        
        while (timer.seconds() < maxAimTime && !shotComplete) {
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
            
            // Check if we're aimed and can shoot
            if (robot.limelight.hasTarget() && !launchSequenceStarted) {
                double aimError = Math.abs(robot.limelight.getTx());
                
                if (aimError <= aimTolerance && timer.seconds() > 1.0) { // Allow 1 second minimum aim time
                    boolean sequenceStarted = robot.startLaunchSequence();
                    if (sequenceStarted) {
                        launchSequenceStarted = true;
                    }
                }
            }
            
            // Check if launch sequence is complete
            if (launchSequenceStarted && !robot.launchSequenceController.isRunning()) {
                shotComplete = true;
            }
            
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                break;
            }
        }
        
        // Stop automated systems
        if (robot.getAimingController() != null) {
            robot.getAimingController().stopAiming();
            robot.turret.setState(Turret.TurretState.MANUAL);
        }
        
        if (robot.getShootingController() != null) {
            robot.getShootingController().setAutoShootingEnabled(false);
        }
        
        return launchSequenceStarted;
    }
    
    /**
     * Waits for the launch sequence to complete.
     * @param robot The robot instance
     * @param maxWaitTime Maximum time to wait in seconds
     * @return true if sequence completed, false if timed out
     */
    public static boolean waitForLaunchComplete(Robot robot, double maxWaitTime) {
        ElapsedTime timer = new ElapsedTime();
        
        while (timer.seconds() < maxWaitTime) {
            if (!robot.launchSequenceController.isRunning()) {
                return true; // Sequence completed
            }
            
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                return false;
            }
        }
        
        return false; // Timed out
    }
    
    // =================================================================================
    // STRATEGY UTILITIES
    // =================================================================================
    
    /**
     * Gets the optimal starting position based on alliance color and strategy.
     * @param alliance "RED" or "BLUE"
     * @param strategy "LEFT" or "RIGHT" starting position
     * @return Starting pose
     */
    public static Pose getStartingPosition(String alliance, String strategy) {
        if (alliance.equalsIgnoreCase("BLUE")) {
            return strategy.equalsIgnoreCase("LEFT") ? FieldPositions.BLUE_START_LEFT : FieldPositions.BLUE_START_RIGHT;
        } else {
            return strategy.equalsIgnoreCase("LEFT") ? FieldPositions.RED_START_LEFT : FieldPositions.RED_START_RIGHT;
        }
    }
    
    /**
     * Gets the optimal shooting position based on alliance color.
     * @param alliance "RED" or "BLUE"
     * @return Shooting position pose
     */
    public static Pose getShootingPosition(String alliance) {
        return alliance.equalsIgnoreCase("BLUE") ? FieldPositions.BLUE_SHOOTING_POSITION : FieldPositions.RED_SHOOTING_POSITION;
    }
    
    /**
     * Gets the parking position based on alliance color.
     * @param alliance "RED", "BLUE", or "NEUTRAL"
     * @return Parking pose
     */
    public static Pose getParkingPosition(String alliance) {
        switch (alliance.toUpperCase()) {
            case "BLUE":
                return FieldPositions.BLUE_PARK;
            case "RED":
                return FieldPositions.RED_PARK;
            default:
                return FieldPositions.NEUTRAL_PARK;
        }
    }
    
    // =================================================================================
    // TIMING UTILITIES
    // =================================================================================
    
    /**
     * Calculates remaining autonomous time.
     * @param autoTimer Timer started at beginning of autonomous
     * @param maxAutoTime Maximum autonomous time (usually 30 seconds)
     * @return Remaining time in seconds
     */
    public static double getRemainingTime(ElapsedTime autoTimer, double maxAutoTime) {
        return Math.max(0, maxAutoTime - autoTimer.seconds());
    }
    
    /**
     * Checks if there's enough time left for a given action.
     * @param autoTimer Timer started at beginning of autonomous
     * @param actionTime Time required for the action
     * @param maxAutoTime Maximum autonomous time
     * @param safetyBuffer Safety buffer to ensure completion
     * @return true if there's enough time
     */
    public static boolean hasTimeFor(ElapsedTime autoTimer, double actionTime, double maxAutoTime, double safetyBuffer) {
        double timeNeeded = actionTime + safetyBuffer;
        double timeRemaining = getRemainingTime(autoTimer, maxAutoTime);
        return timeRemaining >= timeNeeded;
    }
}
