package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.config.Constants;

/**
 * Controller class for automatic distance-based shooting.
 * Integrates Limelight vision distance measurements with Shooter automatic parameters.
 * This class acts as the "brain" for the automatic shooting system.
 */
public class ShootingController {
    
    // =================================================================================
    // SUBSYSTEM REFERENCES
    // =================================================================================
    
    private final Shooter shooter;
    private final Limelight limelight;
    
    // =================================================================================
    // STATE VARIABLES
    // =================================================================================
    
    private boolean autoShootingEnabled = false;
    private double lastValidDistance = 0.0;
    private long lastDistanceUpdate = 0;
    private long lastParameterUpdate = 0;
    
    // Distance measurement stability tracking
    private double[] distanceHistory = new double[5]; // Rolling average for stability
    private int distanceHistoryIndex = 0;
    private boolean distanceHistoryFull = false;
    
    // =================================================================================
    // CONSTRUCTOR
    // =================================================================================
    
    /**
     * Creates a new ShootingController.
     * @param shooter The Shooter subsystem to control
     * @param limelight The Limelight subsystem for distance measurements
     */
    public ShootingController(Shooter shooter, Limelight limelight) {
        this.shooter = shooter;
        this.limelight = limelight;
        
        // Initialize distance history array
        for (int i = 0; i < distanceHistory.length; i++) {
            distanceHistory[i] = 0.0;
        }
    }
    
    // =================================================================================
    // MAIN CONTROL METHODS
    // =================================================================================
    
    /**
     * Enables or disables automatic shooting mode.
     * When enabled, the shooter will use distance-based parameters.
     * @param enabled true to enable auto shooting, false to disable
     */
    public void setAutoShootingEnabled(boolean enabled) {
        if (autoShootingEnabled != enabled) {
            autoShootingEnabled = enabled;
            
            if (enabled) {
                // Switch shooter to automatic mode
                shooter.setState(Shooter.ShooterState.AUTOMATIC);
            } else {
                // Switch shooter back to manual mode
                shooter.setState(Shooter.ShooterState.MANUAL);
            }
        }
    }
    
    /**
     * Main update method - call this in your TeleOp loop.
     * Updates distance measurements and shooting parameters if auto shooting is enabled.
     */
    public void update() {
        if (!autoShootingEnabled) {
            return; // Do nothing if auto shooting is disabled
        }
        
        // Update distance measurement
        updateDistanceMeasurement();
        
        // Update shooting parameters if we have a valid distance
        if (hasValidDistance()) {
            updateShootingParameters();
        }
    }
    
    /**
     * Updates the distance measurement from the Limelight.
     * Uses a rolling average for stability and noise reduction.
     */
    private void updateDistanceMeasurement() {
        if (System.currentTimeMillis() - lastDistanceUpdate < Constants.AutoShootingConfig.DISTANCE_MEASUREMENT_TIMEOUT_MS) {
            return; // Don't update too frequently
        }
        
        if (limelight.hasTarget()) {
            // Get distance from Limelight
            double currentDistance = limelight.getDistanceToTarget();
            
            // Validate distance is within reasonable range
            if (currentDistance >= Constants.AutoShootingConfig.MIN_SHOOTING_DISTANCE &&
                currentDistance <= Constants.AutoShootingConfig.MAX_SHOOTING_DISTANCE) {
                
                // Add to rolling average
                distanceHistory[distanceHistoryIndex] = currentDistance;
                distanceHistoryIndex = (distanceHistoryIndex + 1) % distanceHistory.length;
                
                if (distanceHistoryIndex == 0) {
                    distanceHistoryFull = true;
                }
                
                // Calculate averaged distance
                lastValidDistance = calculateAverageDistance();
                lastDistanceUpdate = System.currentTimeMillis();
            }
        }
    }
    
    /**
     * Calculates the average distance from the rolling history buffer.
     * @return The averaged distance
     */
    private double calculateAverageDistance() {
        double sum = 0.0;
        int count = distanceHistoryFull ? distanceHistory.length : distanceHistoryIndex;
        
        for (int i = 0; i < count; i++) {
            sum += distanceHistory[i];
        }
        
        return count > 0 ? sum / count : 0.0;
    }
    
    /**
     * Updates the shooter's target distance and parameters.
     */
    private void updateShootingParameters() {
        if (System.currentTimeMillis() - lastParameterUpdate < Constants.AutoShootingConfig.PARAMETER_UPDATE_DELAY_MS) {
            return; // Don't update too frequently
        }
        
        // Set the target distance in the shooter - this triggers parameter calculation
        shooter.setTargetDistance(lastValidDistance);
        
        // Tell the shooter to apply the automatic parameters
        shooter.updateAutomatic();
        
        lastParameterUpdate = System.currentTimeMillis();
    }
    
    // =================================================================================
    // UTILITY METHODS
    // =================================================================================
    
    /**
     * Checks if we have a valid distance measurement.
     * @return true if we have a recent, valid distance measurement
     */
    public boolean hasValidDistance() {
        return lastValidDistance > 0 && 
               (System.currentTimeMillis() - lastDistanceUpdate) < (Constants.AutoShootingConfig.DISTANCE_MEASUREMENT_TIMEOUT_MS * 3);
    }
    
    /**
     * Checks if the Limelight currently has a target.
     * @return true if Limelight sees a target
     */
    public boolean hasTarget() {
        return limelight.hasTarget();
    }
    
    /**
     * Forces an immediate parameter update regardless of timing constraints.
     * Useful for manual triggering or testing.
     */
    public void forceParameterUpdate() {
        if (hasValidDistance()) {
            shooter.setTargetDistance(lastValidDistance);
            shooter.updateAutomatic();
            lastParameterUpdate = System.currentTimeMillis();
        }
    }
    
    /**
     * Resets the distance measurement history.
     * Useful when switching targets or when the robot moves significantly.
     */
    public void resetDistanceHistory() {
        for (int i = 0; i < distanceHistory.length; i++) {
            distanceHistory[i] = 0.0;
        }
        distanceHistoryIndex = 0;
        distanceHistoryFull = false;
        lastValidDistance = 0.0;
    }
    
    // =================================================================================
    // GETTER METHODS
    // =================================================================================
    
    /**
     * Gets the current auto shooting enabled state.
     * @return true if auto shooting is enabled
     */
    public boolean isAutoShootingEnabled() {
        return autoShootingEnabled;
    }
    
    /**
     * Gets the last valid distance measurement.
     * @return The distance in inches
     */
    public double getLastValidDistance() {
        return lastValidDistance;
    }
    
    /**
     * Gets the current shooter state.
     * @return The current ShooterState
     */
    public Shooter.ShooterState getShooterState() {
        return shooter.getState();
    }
    
    /**
     * Checks if the automatic shooting system is ready to fire.
     * @return true if auto shooting is enabled, has valid distance, and shooter is ready
     */
    public boolean isReadyToShoot() {
        return autoShootingEnabled && 
               hasValidDistance() && 
               shooter.isAutoShootingReady() && 
               hasTarget();
    }
    
    /**
     * Gets a status string for telemetry display.
     * @return Formatted status string
     */
    public String getStatusString() {
        if (!autoShootingEnabled) {
            return "Auto Shooting: DISABLED";
        }
        
        if (!hasTarget()) {
            return "Auto Shooting: NO TARGET";
        }
        
        if (!hasValidDistance()) {
            return "Auto Shooting: MEASURING...";
        }
        
        if (isReadyToShoot()) {
            return "Auto Shooting: READY (" + shooter.getShootingParametersString() + ")";
        }
        
        return "Auto Shooting: CALCULATING...";
    }
    
    // =================================================================================
    // DIRECT ACCESS METHODS (for advanced users)
    // =================================================================================
    
    /**
     * Gets direct access to the Shooter subsystem.
     * @return The Shooter instance
     */
    public Shooter getShooter() {
        return shooter;
    }
    
    /**
     * Gets direct access to the Limelight subsystem.
     * @return The Limelight instance
     */
    public Limelight getLimelight() {
        return limelight;
    }
}
