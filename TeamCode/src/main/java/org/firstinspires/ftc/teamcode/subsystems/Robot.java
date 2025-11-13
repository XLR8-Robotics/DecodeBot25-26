package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.teamcode.config.FieldConstants;
import org.firstinspires.ftc.teamcode.utils.PatternIdentifier;
import org.firstinspires.ftc.teamcode.utils.ShooterTable;

import java.util.List;

/**
 * Main Robot class that manages all subsystems and provides both simple manual control
 * and automated launch sequences. Supports both legacy and enhanced aiming modes.
 */
public class Robot {

    // =================================================================================
    // SUBSYSTEM INSTANCES
    // =================================================================================
    
    // Core subsystems
    public BasicDriveTrain basicDriveTrain;
    public Intake intake;
    public Shooter shooter;
    public Turret turret;
    public Limelight limelight;
    
    // Advanced motion control (optional - for enhanced aiming)
    public Follower follower;
    public AimingController aimingController;
    
    // Advanced shooting control (optional - for automatic shooting)
    public ShootingController shootingController;
    
    // Launch sequence control (cleaner implementation)
    public LaunchSequenceController launchSequenceController;

    // =================================================================================
    // ENUMS AND CONFIGURATION
    // =================================================================================
    
    /** Enum for target side selection */
    public enum TargetSide {
        RED,
        BLUE
    }
    
    /** Enum for aiming system mode */
    public enum AimingMode {
        LEGACY,    // Original PID-based aiming (backward compatibility)
        ENHANCED   // New AimingController with pedropathing integration
    }
    
    /** Enum for shooting system mode */
    public enum ShootingMode {
        MANUAL,    // Manual control only (backward compatibility)
        AUTOMATIC  // Automatic distance-based shooting with ShootingController
    }

    // Launch sequence state is now managed by LaunchSequenceController
    
    // =================================================================================
    // PRIVATE STATE VARIABLES
    // =================================================================================
    
    // Robot configuration
    private TargetSide currentTargetSide = TargetSide.BLUE;
    private AimingMode currentAimingMode = AimingMode.LEGACY;
    private ShootingMode currentShootingMode = ShootingMode.MANUAL;
    
    // =================================================================================
    // CONSTRUCTORS
    // =================================================================================

    /**
     * Default constructor for backward compatibility.
     * Uses legacy aiming mode with basic PID control.
     */
    public Robot() {
        this.currentAimingMode = AimingMode.LEGACY;
    }
    
    /**
     * Enhanced constructor with pedropathing integration.
     * @param follower The pedropathing Follower for odometry and motion control
     */
    public Robot(Follower follower) {
        this.follower = follower;
        this.currentAimingMode = AimingMode.ENHANCED;
    }
    
    /**
     * Legacy initialization method for backward compatibility.
     * Initializes all subsystems without advanced aiming.
     * @param hardwareMap The hardware map from the OpMode
     */
    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, AimingMode.LEGACY);
    }
    
    /**
     * Enhanced initialization method with aiming mode selection.
     * @param hardwareMap The hardware map from the OpMode
     * @param aimingMode The aiming system mode to use
     */
    public void init(HardwareMap hardwareMap, AimingMode aimingMode) {
        init(hardwareMap, aimingMode, ShootingMode.MANUAL);
    }
    
    /**
     * Complete initialization method with both aiming and shooting mode selection.
     * @param hardwareMap The hardware map from the OpMode
     * @param aimingMode The aiming system mode to use
     * @param shootingMode The shooting system mode to use
     */
    public void init(HardwareMap hardwareMap, AimingMode aimingMode, ShootingMode shootingMode) {
        // Initialize basic subsystems
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap);
        basicDriveTrain = new BasicDriveTrain(hardwareMap);
        
        // Initialize launch sequence controller (always available)
        launchSequenceController = new LaunchSequenceController(
            intake, shooter, turret, limelight, basicDriveTrain);
        
        // Set modes
        this.currentAimingMode = aimingMode;
        this.currentShootingMode = shootingMode;
        
        // Initialize enhanced aiming system if requested and follower is available
        if (aimingMode == AimingMode.ENHANCED && follower != null) {
            aimingController = new AimingController(turret, limelight, follower);
        }
        
        // Initialize automatic shooting system if requested
        if (shootingMode == ShootingMode.AUTOMATIC) {
            shootingController = new ShootingController(shooter, limelight);
        }
    }
    
    /**
     * Complete initialization method for enhanced mode.
     * @param hardwareMap The hardware map from the OpMode
     * @param follower The pedropathing Follower for odometry
     */
    public void init(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;
        init(hardwareMap, AimingMode.ENHANCED, ShootingMode.MANUAL);
    }
    
    /**
     * Complete initialization method for enhanced mode with automatic shooting.
     * @param hardwareMap The hardware map from the OpMode
     * @param follower The pedropathing Follower for odometry
     * @param shootingMode The shooting system mode to use
     */
    public void init(HardwareMap hardwareMap, Follower follower, ShootingMode shootingMode) {
        this.follower = follower;
        init(hardwareMap, AimingMode.ENHANCED, shootingMode);
    }

    // =================================================================================
    // PUBLIC UPDATE METHODS
    // =================================================================================

    /**
     * Main update method with launch sequence support.
     * Use this when you want the launch sequence functionality.
     */
    public void update(Gamepad gamepad) {
        limelight.update();
        
        // Update automatic shooting system if enabled
        if (shootingController != null) {
            shootingController.update();
        }

        // Update launch sequence controller
        launchSequenceController.update(gamepad);

        // If the launch sequence is not active, allow manual control.
        if (!launchSequenceController.isRunning()) {
            updateDrivetrainControl(gamepad);
            updateSubsystemsManualControl(gamepad);
        }
    }

    /**
     * Pure manual control update method - NO launch sequence.
     * Use this for simple manual control without automated sequences.
     */
    public void manualUpdate(Gamepad gamepad) {
        limelight.update();
        
        // Update automatic shooting system if enabled
        if (shootingController != null) {
            shootingController.update();
        }
        
        updateDrivetrainControl(gamepad);
        updateSubsystemsManualControl(gamepad);
    }

    // =================================================================================
    // PRIVATE HELPER METHODS
    // =================================================================================
    
    /**
     * Updates drivetrain control based on gamepad input.
     * @param gamepad The gamepad to read input from
     */
    private void updateDrivetrainControl(Gamepad gamepad) {
        double forward = -gamepad.left_stick_y * Constants.DrivetrainConfig.DRIVE_SPEED_MULTIPLIER;
        double strafe = gamepad.left_stick_x * Constants.DrivetrainConfig.DRIVE_SPEED_MULTIPLIER;
        double turn = gamepad.right_stick_x * Constants.DrivetrainConfig.DRIVE_SPEED_MULTIPLIER;
        basicDriveTrain.drive(forward, strafe, turn);
    }
    
    /**
     * Updates all subsystems with manual control.
     * @param gamepad The gamepad to read input from
     */
    private void updateSubsystemsManualControl(Gamepad gamepad) {
        intake.update(gamepad);
        shooter.update(gamepad);
        turret.manualUpdate(gamepad);
    }

    // =================================================================================
    // LAUNCH SEQUENCE METHODS (now delegating to LaunchSequenceController)
    // =================================================================================

    /**
     * Starts the launch sequence.
     * @return true if sequence was started, false if already running
     */
    public boolean startLaunchSequence() {
        return launchSequenceController.startSequence();
    }

    /**
     * Cancels the current launch sequence.
     * @return true if sequence was cancelled, false if not running
     */
    public boolean cancelLaunchSequence() {
        return launchSequenceController.cancelSequence();
    }

    /**
     * Emergency stop for the launch sequence.
     */
    public void emergencyStopLaunchSequence() {
        launchSequenceController.emergencyStop();
    }

    /**
     * Stops all motors safely.
     */
    public void stopAllMotors() {
        if (launchSequenceController != null) {
            launchSequenceController.emergencyStop();
        } else {
            // Fallback for when controller isn't initialized yet
            basicDriveTrain.drive(0, 0, 0);
            if (intake != null) intake.setPower(0);
            if (shooter != null) shooter.setPower(0);
            if (turret != null) turret.setPower(0);
        }
    }

    // =================================================================================
    // PUBLIC GETTER/SETTER METHODS
    // =================================================================================

    /**
     * Gets the current launch sequence state as a string.
     * @return Current launch sequence state description
     */
    public String getLaunchSequenceState() {
        return launchSequenceController != null 
            ? launchSequenceController.getCurrentState().toString()
            : "NOT_INITIALIZED";
    }

    /**
     * Gets detailed launch sequence status for telemetry.
     * @return Formatted status string with state and timing
     */
    public String getLaunchSequenceStatus() {
        return launchSequenceController != null 
            ? launchSequenceController.getStatusString()
            : "Launch Controller Not Initialized";
    }

    public TargetSide getTargetSide() {
        return currentTargetSide;
    }

    public void setTargetSide(TargetSide side) {
        this.currentTargetSide = side;
    }

    public int getCurrentTargetTagId() {
        if (currentAimingMode == AimingMode.ENHANCED) {
            // DECODE season AprilTag IDs
            return (currentTargetSide == TargetSide.RED) 
                ? FieldConstants.RED_APRILTAG_ID 
                : FieldConstants.BLUE_APRILTAG_ID;
        } else {
            // Legacy mode using PatternIdentifier
            return (currentTargetSide == TargetSide.RED)
                    ? PatternIdentifier.TOWER_RED
                    : PatternIdentifier.TOWER_BLUE;
        }
    }
    
    /**
     * Get the current aiming mode.
     * @return The current AimingMode (LEGACY or ENHANCED)
     */
    public AimingMode getAimingMode() {
        return currentAimingMode;
    }
    
    
    /**
     * Get the pedropathing Follower (if using enhanced mode).
     * @return The Follower instance, or null if in legacy mode
     */
    public Follower getFollower() {
        return follower;
    }
    
    /**
     * Get the AimingController (if using enhanced mode).
     * @return The AimingController instance, or null if in legacy mode
     */
    public AimingController getAimingController() {
        return aimingController;
    }
    
    /**
     * Get the current shooting system mode.
     * @return The current ShootingMode
     */
    public ShootingMode getShootingMode() {
        return currentShootingMode;
    }
    
    /**
     * Get the ShootingController (if using automatic shooting mode).
     * @return The ShootingController instance, or null if in manual mode
     */
    public ShootingController getShootingController() {
        return shootingController;
    }
    
    /**
     * Set the shooting system mode.
     * @param shootingMode The new shooting mode
     */
    public void setShootingMode(ShootingMode shootingMode) {
        if (currentShootingMode != shootingMode) {
            currentShootingMode = shootingMode;
            
            if (shootingMode == ShootingMode.AUTOMATIC && shootingController == null) {
                // Create shooting controller if it doesn't exist
                shootingController = new ShootingController(shooter, limelight);
            }
            
            // Update shooter state based on mode
            if (shootingController != null) {
                shootingController.setAutoShootingEnabled(shootingMode == ShootingMode.AUTOMATIC);
            }
        }
    }
}
