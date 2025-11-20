package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.FieldConstants;
import org.firstinspires.ftc.teamcode.utils.PatternIdentifier;


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

    // Launch sequence state is now managed by LaunchSequenceController
    
    // =================================================================================
    // PRIVATE STATE VARIABLES
    // =================================================================================
    
    // Robot configuration
    private TargetSide currentTargetSide = TargetSide.BLUE;
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
    public Follower getFollower() {
        return follower;
    }
}
