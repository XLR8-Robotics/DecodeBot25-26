package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Subsystem for interfacing with the Limelight 3A camera and its AprilTag detection.
 */
public class Limelight {

    private final Limelight3A limelight;
    private LLResult latestResult = null;

    public Limelight(HardwareMap hardwareMap) {

        limelight = hardwareMap.get(Limelight3A.class, Constants.HardwareConfig.LIMELIGHT_NAME);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    /**
     * This method should be called in a loop to update the detection data.
     */
    public void update() {
        latestResult = limelight.getLatestResult();
    }

    /**
     * Updates the robot's orientation in the Limelight for MegaTag 2 fusion.
     * @param headingDegrees The robot's heading in degrees.
     */
    public void updateRobotOrientation(double headingDegrees) {
        if (limelight != null) {
            limelight.updateRobotOrientation(headingDegrees);
        }
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
     * Calculates the robot's field pose based on vision data and a known AprilTag position.
     * This method works backward from the known tag position using the camera's measurements.
     * @param knownTagPose The known field position of the detected AprilTag
     * @return The calculated robot pose, or null if no target is visible or calculation fails
     */
    public com.pedropathing.geometry.Pose getVisionBasedRobotPose(com.pedropathing.geometry.Pose knownTagPose) {
        if (!hasTarget() || knownTagPose == null) {
            return null;
        }
        
        try {
            // Get vision measurements
            double tx = getTx(); // Horizontal offset in degrees
            double distance = getDistanceToTarget(); // Distance to tag in inches
            
            if (distance <= 0) {
                return null; // Invalid distance measurement
            }
            
            // Convert tx to radians
            double txRadians = Math.toRadians(tx);
            
            // Calculate the angle from the tag to the robot in field coordinates
            // The camera sees the tag at angle tx, so the robot is at angle (tag_heading + tx + 180°) from the tag
            double angleFromTagToRobot = knownTagPose.getHeading() + txRadians + Math.PI;
            
            // Calculate robot position relative to the tag
            double robotX = knownTagPose.getX() + distance * Math.cos(angleFromTagToRobot);
            double robotY = knownTagPose.getY() + distance * Math.sin(angleFromTagToRobot);
            
            // The robot's heading is approximately the angle it's looking at the tag
            // This is a simplified calculation - more sophisticated approaches would use
            // the robot's IMU or multiple measurements for better accuracy
            double robotHeading = angleFromTagToRobot + Math.PI - txRadians;
            
            // Normalize the heading to [-π, π]
            while (robotHeading > Math.PI) {
                robotHeading -= 2 * Math.PI;
            }
            while (robotHeading < -Math.PI) {
                robotHeading += 2 * Math.PI;
            }
            
            return new com.pedropathing.geometry.Pose(robotX, robotY, robotHeading);
            
        } catch (Exception e) {
            // Return null if any calculation fails
            return null;
        }
    }

    /**
     * Returns the robot's Pose3D relative to the field (via MegaTag or similar), if available from Limelight.
     * This relies on the Limelight pipeline being configured for 3D/Robot Pose.
     * @return The Pose3D of the robot, or null if not available.
     */
    public Pose3D getRobotPose3D() {
        if (latestResult == null || !latestResult.isValid()) {
            return null;
        }
        // Limelight 3A/MegaTag results usually provide a botpose
        return latestResult.getBotpose();
    }

    /**
     * Calculates the robot's heading from the Limelight's 3D pose estimation (MegaTag).
     * @return The robot's heading in degrees, or Double.NaN if invalid.
     */
    public double getVisionHeading() {
        Pose3D pose = getRobotPose3D();
        if (pose != null) {
            // The orientation is usually returned as a quaternion or Euler angles.
            // LLResult botpose is in Field Space.
            // Yaw is usually the rotation around the vertical axis (Z or Y depending on coord system).
            // Limelight usually returns [x, y, z, roll, pitch, yaw] in the networktables array, 
            // but the SDK object might expose it differently.
            // Pose3D in FTC SDK has getOrientation().
            
            // Let's assume standard field centric yaw.
            // We need to be careful about units and coordinate systems.
            // Limelight docs say Yaw is in degrees.
            
             return pose.getOrientation().getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);
        }
        return Double.NaN;
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
