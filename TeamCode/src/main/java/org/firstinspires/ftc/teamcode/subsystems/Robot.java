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
    public Robot(HardwareMap hardwareMap) {
        this.basicDriveTrain = new BasicDriveTrain(hardwareMap);
        this.intake = new Intake(hardwareMap);
        this.shooter = new Shooter(hardwareMap);
        this.turret = new Turret(hardwareMap);
        this.limelight = new Limelight(hardwareMap);
        this.launchSequenceController = new LaunchSequenceController(intake, shooter, turret, limelight, basicDriveTrain);
    }


    public void update(Gamepad gamepad) {
        limelight.update();

        launchSequenceController.update(gamepad);

        // If the launch sequence is not active, allow manual control.
        if (!launchSequenceController.isRunning()) {
            updateDrivetrainControl(gamepad);
            updateSubsystemsManualControl(gamepad);
        }
    }

    // =================================================================================
    // PRIVATE HELPER METHODS
    // =================================================================================
    
    /**
     * Updates drivetrain control based on gamepad input.
     * @param gamepad The gamepad to read input from
     */
    private void updateDrivetrainControl(Gamepad gamepad) {
        // Standard mecanum drive control: left stick for translation, right stick for rotation

        basicDriveTrain.drive(-gamepad.left_stick_y, gamepad.right_stick_x, gamepad.left_stick_x);
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
            // Fallback for when controller isn'''t initialized yet
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
                // Create shooting controller if it doesn'''t exist
                shootingController = new ShootingController(shooter, limelight);
            }
            
            // Update shooter state based on mode
            if (shootingController != null) {
                shootingController.setAutoShootingEnabled(shootingMode == ShootingMode.AUTOMATIC);
            }
        }
    }
}
