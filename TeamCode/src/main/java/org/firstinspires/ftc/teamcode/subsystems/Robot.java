package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
     * REQUIRED: motor names must match your RC configuration.
     */
    public Robot(HardwareMap hardwareMap,
                 String leftFrontName, String leftRearName,
                 String rightFrontName, String rightRearName) {
        this.hardwareMap = hardwareMap;
        this.driveTrain = new DriveTrain(hardwareMap, leftFrontName, leftRearName, rightFrontName, rightRearName);
        this.controls = new Controls();
        this.driveTrain.setControls(controls);
        this.limelight = new Limelight();
    }

    /**
     * Configure odometry (Pinpoint) and auto-attach Pedro localizer if present.
     */
    public void configureOdometry(String deviceName,
                                  double xPodOffsetMm,
                                  double yPodOffsetMm,
                                  boolean forwardEncoderForward,
                                  boolean strafeEncoderForward,
                                  boolean useFourBarPod) {
        driveTrain.configureOdometry(deviceName, xPodOffsetMm, yPodOffsetMm,
                forwardEncoderForward, strafeEncoderForward, useFourBarPod);
    }

    /**
     * Call in TeleOp start.
     */
    public void startTeleOp() {
        driveTrain.startTeleOpDrive();
    }

    /**
     * Call each TeleOp loop. Scales control the max output and precision mode.
     */
    public void teleopLoop(Gamepad gamepad1, double normalScale, double slowScale) {
        driveTrain.teleop(gamepad1, normalScale, slowScale);
    }

    public DriveTrain getDriveTrain() { return driveTrain; }
    public Controls getControls() { return controls; }
    public HardwareMap getHardwareMap() { return hardwareMap; }
    public Limelight getLimelight() { return limelight; }
}


