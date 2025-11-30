package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.Constants;
import java.util.List;

/**
 * Limelight 3A subsystem using the official FTC SDK Limelight3A class.
 */
public class Limelight {
    private final Limelight3A limelightHardware;
    private LLResult lastResult = null;

    public Limelight(HardwareMap hardwareMap) {
        if (hardwareMap == null) {
            throw new IllegalArgumentException("HardwareMap cannot be null.");
        }

        limelightHardware = hardwareMap.get(Limelight3A.class, Constants.HardwareConfig.LIMELIGHT_NAME);
    }

    /**
     * Starts polling for data from the Limelight.
     * Call this once, e.g., in OpMode init or Robot constructor.
     */
    public void start() {
        if (limelightHardware != null) {
            limelightHardware.start();
        }
    }

    /**
     * Stops polling for data from the Limelight.
     * Call this when the OpMode ends.
     */
    public void stop() {
        if (limelightHardware != null) {
            limelightHardware.stop();
        }
    }

    /**
     * Switches the Limelight's active pipeline.
     * @param pipelineIndex The index of the pipeline to switch to (0-9).
     */
    public void pipelineSwitch(int pipelineIndex) {
        if (limelightHardware != null) {
            limelightHardware.pipelineSwitch(pipelineIndex);
        }
    }

    /**
     * Gets the current status of the Limelight.
     * @return LLStatus object, or null if hardware not initialized.
     */
    public LLStatus getStatus() {
        return limelightHardware != null ? limelightHardware.getStatus() : null;
    }

    /**
     * Fetches and stores the latest result from the Limelight.
     * This should be called periodically (e.g., in your main loop) to get fresh data.
     */
    public void updateResult() {
        if (limelightHardware != null) {
            lastResult = limelightHardware.getLatestResult();
        }
    }

    /**
     * Returns the most recently fetched result.
     * Call updateResult() before this to ensure data is fresh.
     * @return LLResult object, or null if no result available or hardware not initialized.
     */
    public LLResult getLatestResult() {
        return lastResult;
    }

    /**
     * Checks if the last fetched result is valid and contains targets.
     * @return true if targets are present, false otherwise.
     */
    public boolean hasTargets() {
        return lastResult != null && lastResult.isValid();
    }

    /**
     * Gets the ID of the best/first detected AprilTag fiducial from the last fetched result.
     * @return The ID of the first fiducial, or -1 if no fiducials are detected or result is invalid.
     */
    public int getBestTagId() {
        if (hasTargets()) {
            List<LLResultTypes.FiducialResult> fiducialResults = lastResult.getFiducialResults();
            if (fiducialResults != null && !fiducialResults.isEmpty()) {
                return fiducialResults.get(0).getFiducialId();
            }
        }
        return -1;
    }

    /**
     * Gets the target's crosshair-relative horizontal offset from the last fetched result.
     * @return tx value, or 0.0 if no valid target.
     */
    public double getTx() {
        return hasTargets() ? lastResult.getTx() : 0.0;
    }

    /**
     * Gets the target's crosshair-relative vertical offset from the last fetched result.
     * @return ty value, or 0.0 if no valid target.
     */
    public double getTy() {
        return hasTargets() ? lastResult.getTy() : 0.0;
    }

    /**
     * Gets the robot's pose in the AprilTag field coordinate system from the last fetched result.
     * This is the {@code botpose_targetspace} result from the Limelight.
     * @return Pose3D object, or null if no valid botpose is available.
     */
    public Pose3D getBotPose() {
        return hasTargets() ? lastResult.getBotpose() : null;
    }

    /**
     * Returns a human-friendly name for the detected tag id based on TuningConfig mapping.
     * @return The friendly name, "None" if no tag, or "Tag [id]" if unmapped.
     */
    public String getBestTagName() {
        int tagId = getBestTagId();

        if(tagId == 20 ){
            return "Blue";
        }

        if(tagId == 24){
            return "Red";
        }

        return "None";
    }
}