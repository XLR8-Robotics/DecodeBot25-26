package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.geometry.Pose;
import java.util.Map;
import java.util.HashMap;

/**
 * Field constants for the DECODE season AprilTag positions.
 * Official coordinates for the 2025-2026 DECODE season.
 */
public class FieldConstants {
    
    // DECODE season AprilTag IDs
    public static final int RED_APRILTAG_ID = 20;
    public static final int BLUE_APRILTAG_ID = 24;
    
    /**
     * Map of AprilTag IDs to their field positions using pedropathing Pose.
     * Coordinates are in inches with (0,0) at field center.
     * Heading is in radians.
     * 
     * DECODE season has two AprilTags positioned in opposite corners of the field.
     */
    public static final Map<Integer, Pose> APRIL_TAG_POSITIONS = new HashMap<>();

    static {
        // DECODE season AprilTag positions (opposite corners)
        
        // Red AprilTag - positioned at (57, 57) facing 120 degrees
        APRIL_TAG_POSITIONS.put(RED_APRILTAG_ID, new Pose(57.0, 57.0, Math.toRadians(120)));
        
        // Blue AprilTag - positioned at (57, -57) facing 270 degrees  
        APRIL_TAG_POSITIONS.put(BLUE_APRILTAG_ID, new Pose(57.0, -57.0, Math.toRadians(270)));
    }
    
    /**
     * Get the field position of an AprilTag by its ID.
     * @param id The AprilTag ID
     * @return The Pose of the AprilTag, or null if the ID is not found
     */
    public static Pose getAprilTagPose(int id) {
        return APRIL_TAG_POSITIONS.get(id);
    }
    
    /**
     * Check if an AprilTag ID exists in our field constants.
     * @param id The AprilTag ID to check
     * @return true if the ID exists, false otherwise
     */
    public static boolean hasAprilTag(int id) {
        return APRIL_TAG_POSITIONS.containsKey(id);
    }
}
