package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.config.FieldConstants;

/**
 * AimingController manages the automatic turret aiming system.
 * It coordinates between the Turret, Limelight, and Follower to aim at AprilTags.
 */
public class AimingController {
    private final Turret turret;
    private final Limelight limelight;
    private final Follower follower;
    
    private int targetAprilTagId = -1;
    
    /**
     * Constructor for the AimingController.
     * @param turret The turret subsystem
     * @param limelight The limelight vision subsystem
     * @param follower The pedropathing follower for odometry
     */
    public AimingController(Turret turret, Limelight limelight, Follower follower) {
        this.turret = turret;
        this.limelight = limelight;
        this.follower = follower;
    }
    
    /**
     * Set the target AprilTag ID to aim at.
     * @param id The AprilTag ID to target
     */
    public void setTargetAprilTagId(int id) {
        this.targetAprilTagId = id;
    }
    
    /**
     * Get the current target AprilTag ID.
     * @return The current target AprilTag ID, or -1 if no target is set
     */
    public int getTargetAprilTagId() {
        return targetAprilTagId;
    }
    
    /**
     * Main update method that performs the aiming logic.
     * This method should be called regularly in the main loop when aiming mode is active.
     */
    public void update() {
        // Early return if no target is set
        if (targetAprilTagId == -1) {
            return;
        }
        
        // i. Get the robot's current Pose from the Follower
        Pose robotPose = follower.getPose();
        
        // ii. Get the target AprilTag's Pose from FieldConstants
        Pose targetPose = FieldConstants.getAprilTagPose(targetAprilTagId);
        if (targetPose == null) {
            // Target AprilTag ID not found in field constants
            return;
        }
        
        // iii. Calculate the required absolute field angle from robot to target
        double deltaX = targetPose.getX() - robotPose.getX();
        double deltaY = targetPose.getY() - robotPose.getY();
        double targetFieldAngle = Math.atan2(deltaY, deltaX);
        
        // iv. Command the turret to this angle
        turret.setTargetFieldAngle(targetFieldAngle);
        
        // v. Update the Limelight and check if the target tag is visible
        limelight.update();
        if (limelight.hasTarget() && limelight.getFiducialId() == targetAprilTagId) {
            // vi. Calculate the robot's pose based on the vision target and correct odometry
            Pose visionBasedPose = limelight.getVisionBasedRobotPose(targetPose);
            if (visionBasedPose != null) {
                follower.setPose(visionBasedPose);
            }
        }
        
        // vii. Call the turret update method with current robot heading in degrees
        turret.update(Math.toDegrees(robotPose.getHeading()));
    }
    
    /**
     * Check if the turret is currently aimed at the target.
     * @return true if aimed at target within tolerance, false otherwise
     */
    public boolean isAimedAtTarget() {
        if (targetAprilTagId == -1) {
            return false;
        }
        
        // This would need to be implemented based on turret's angle tolerance
        // For now, return whether we have a vision target
        return limelight.hasTarget() && limelight.getFiducialId() == targetAprilTagId;
    }
    
    /**
     * Stop aiming and clear the target.
     */
    public void stopAiming() {
        targetAprilTagId = -1;
    }
}
