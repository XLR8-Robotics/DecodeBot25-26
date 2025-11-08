package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.Constants;

import java.util.List;

/**
 * Subsystem for interfacing with the Limelight 3A camera and its AprilTag detection.
 */
public class Limelight {

    private final Limelight3A limelight;
    private LLResult latestResult = null;

    public Limelight(HardwareMap hardwareMap) {
        // Get the Limelight 3A device from the hardware map
        limelight = hardwareMap.get(Limelight3A.class, Constants.HardwareConfig.LIMELIGHT_NAME);
        
        // Set the pipeline to AprilTag detection (typically pipeline 0)
        limelight.pipelineSwitch(0);
        
        // Start polling for data
        limelight.start();
    }

    /**
     * This method should be called in a loop to update the detection data.
     */
    public void update() {
        latestResult = limelight.getLatestResult();
    }

    /**
     * Checks if a valid AprilTag is currently detected.
     * @return true if a tag is detected, false otherwise.
     */
    public boolean hasTarget() {
        if (latestResult == null || !latestResult.isValid()) {
            return false;
        }
        
        // Check if there are any fiducial (AprilTag) detections
        List<LLResultTypes.FiducialResult> fiducialResults = latestResult.getFiducialResults();
        return fiducialResults != null && !fiducialResults.isEmpty();
    }

    /**
     * Gets the ID of the currently detected AprilTag.
     * If multiple tags are detected, returns the first one.
     * @return The ID of the tag, or -1 if no tag is detected.
     */
    public int getFiducialId() {
        if (!hasTarget()) {
            return -1;
        }
        
        List<LLResultTypes.FiducialResult> fiducialResults = latestResult.getFiducialResults();
        if (fiducialResults != null && !fiducialResults.isEmpty()) {
            // Return the first detected fiducial ID
            // Note: If you want to prioritize a specific tag ID when multiple are visible,
            // you can add that logic in the Robot class's handleAutomatedAiming() method
            return fiducialResults.get(0).getFiducialId();
        }
        
        return -1;
    }

    /**
     * Gets all detected fiducial results. Useful when multiple tags are visible.
     * @return A list of detected fiducial results, or an empty list if none are detected.
     */
    public List<LLResultTypes.FiducialResult> getFiducialResults() {
        if (latestResult == null || !latestResult.isValid()) {
            return java.util.Collections.emptyList();
        }
        List<LLResultTypes.FiducialResult> results = latestResult.getFiducialResults();
        return results != null ? results : java.util.Collections.emptyList();
    }

    /**
     * Gets the horizontal offset (tx) of the AprilTag from the center of the screen.
     * This value is essential for the P controller to aim the turret.
     * @return The horizontal offset in degrees. Positive values mean the target is to the right.
     */
    public double getTx() {
        if (!hasTarget()) {
            return 0.0;
        }
        
        // Limelight provides tx directly from the result
        // If there are multiple fiducials, we'll use the first one's position
        List<LLResultTypes.FiducialResult> fiducialResults = latestResult.getFiducialResults();
        if (fiducialResults != null && !fiducialResults.isEmpty()) {
            // Use the targetX from the fiducial result, which is the horizontal offset in degrees
            return fiducialResults.get(0).getTargetXDegrees();
        }
        
        // Fallback to the general tx value from the result
        return latestResult.getTx();
    }

    /**
     * Gets the vertical offset (ty) of the AprilTag from the center of the screen.
     * @return The vertical offset in degrees. Positive values mean the target is above center.
     */
    public double getTy() {
        if (!hasTarget()) {
            return 0.0;
        }
        
        // Limelight provides ty directly from the result
        List<LLResultTypes.FiducialResult> fiducialResults = latestResult.getFiducialResults();
        if (fiducialResults != null && !fiducialResults.isEmpty()) {
            // Use the targetY from the fiducial result, which is the vertical offset in degrees
            return fiducialResults.get(0).getTargetYDegrees();
        }
        
        // Fallback to the general ty value from the result
        return latestResult.getTy();
    }
    /**
     * Calculates the distance to the AprilTag target.
     * @return The distance to the target in inches, or 0.0 if no target is visible.
     */
    public double getDistanceToTarget() {
        if (!hasTarget()) {
            return 0.0;
        }

        double ty = getTy();
        double angle = Constants.LimelightConfig.LIMELIGHT_ANGLE + ty;
        double distance = (Constants.LimelightConfig.APRIL_TAG_HEIGHT - Constants.LimelightConfig.LIMELIGHT_HEIGHT)
                / Math.tan(Math.toRadians(angle));

        return distance;
    }

    /**
     * Stops the Limelight polling. Should be called when done using the Limelight.
     */
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    /**
     * Gets the latest raw result from the Limelight for advanced usage.
     * @return The latest LLResult, or null if no result is available.
     */
    public LLResult getLatestResult() {
        return latestResult;
    }
}
