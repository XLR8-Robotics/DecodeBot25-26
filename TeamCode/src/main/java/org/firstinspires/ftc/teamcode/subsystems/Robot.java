package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.DcMotorConfig;
import org.firstinspires.ftc.teamcode.config.OdometryConfig;
import org.firstinspires.ftc.teamcode.config.LimelightConfig; // Added import

/**
 * Robot container: holds and wires subsystems. Currently includes drivetrain and controls.
 * Extend this class as new subsystems are added (intake, lift, etc.).
 */
public class Robot {
    private final HardwareMap hardwareMap;

    private final DriveTrain driveTrain;
    private final Controls controls;
    private final Limelight limelight;

    /**
     * Legacy constructor using string names for motors. Assumes default Limelight.
     */
    public Robot(HardwareMap hardwareMap,
                 String leftFrontName, String leftRearName,
                 String rightFrontName, String rightRearName) {
        this.hardwareMap = hardwareMap;
        this.driveTrain = new DriveTrain(hardwareMap, leftFrontName, leftRearName, rightFrontName, rightRearName);
        this.controls = new Controls();
        this.driveTrain.setControls(controls);
        // Initialize Limelight with default config
        this.limelight = new Limelight(hardwareMap, LimelightConfig.LIMELIGHT_DEFAULT);
        this.limelight.start(); // Start Limelight polling
    }

    /**
     * Preferred constructor using Config enums.
     */
    public Robot(HardwareMap hardwareMap,
                 DcMotorConfig leftFront,
                 DcMotorConfig leftRear,
                 DcMotorConfig rightFront,
                 DcMotorConfig rightRear,
                 LimelightConfig limelightConfig) { // Added LimelightConfig parameter
        this.hardwareMap = hardwareMap;
        this.driveTrain = new DriveTrain(hardwareMap, leftFront, leftRear, rightFront, rightRear);
        this.controls = new Controls();
        this.driveTrain.setControls(controls);
        this.limelight = new Limelight(hardwareMap, limelightConfig); // Initialize Limelight with provided config
        this.limelight.start(); // Start Limelight polling
    }

    /**
     * Configure odometry (Pinpoint) and auto-attach Pedro localizer if present.
     */
    public void configureOdometry(OdometryConfig config) {
        driveTrain.configureOdometry(config);
    }

    /**
     * Call in TeleOp start.
     */
    public void startTeleOp() {
        driveTrain.startTeleOpDrive();
    }

    /**
     * Call each TeleOp loop. Scales control the max output and precision mode.
     * Also updates Limelight results.
     */
    public void teleopLoop(Gamepad gamepad1, double normalScale, double slowScale) {
        limelight.updateResult(); // Update limelight results each loop
        driveTrain.teleop(gamepad1, normalScale, slowScale);
        // Add Limelight telemetry or processing here if needed
    }
    
    /**
     * Call when OpMode stops to safely stop subsystems like Limelight.
     */
    public void stopRobot() {
        if (limelight != null) {
            limelight.stop();
        }
        // Add any other subsystem stop/cleanup methods here
    }

    public DriveTrain getDriveTrain() { return driveTrain; }
    public Controls getControls() { return controls; }
    public HardwareMap getHardwareMap() { return hardwareMap; }
    public Limelight getLimelight() { return limelight; }
}
