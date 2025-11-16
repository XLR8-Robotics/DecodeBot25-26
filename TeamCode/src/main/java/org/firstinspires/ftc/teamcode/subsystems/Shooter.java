package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.config.Constants;

import java.util.Arrays;
import java.util.List;

/**
 * Subsystem for controlling the shooter mechanism, including the shooter wheel and hood.
 * Supports both manual and automatic distance-based shooting modes.
 */
public class Shooter {
    
    /**
     * Enum representing the different states of the shooter system.
     */
    public enum ShooterState {
        MANUAL,     // Manual control via gamepad
        AUTOMATIC   // Automatic distance-based shooting
    }
    private final DcMotorEx shooterMotor;
    private final Servo hoodServo;
    private boolean isRunning, isIdle;

    
    // State management
    private ShooterState currentState = ShooterState.MANUAL;
    
    // Manual control variables
    private final List<Double> hoodPositions = Arrays.asList(
            Constants.ShooterConfig.HOOD_MIN,
            Constants.ShooterConfig.HOOD_CENTER,
            Constants.ShooterConfig.HOOD_MAX
    );
    private int currentHoodPositionIndex = 2; // Start at center position (index 4)
    private boolean previousDpadUpState = false;
    private boolean previousDpadDownState = false;
    private boolean previousTriangleButtonState = false;
    private boolean isInitialized = false;
    
    // Automatic shooting variables
    private double targetDistance = 0.0;
    private double calculatedPower = 0.0;      // For fallback power control
    private double calculatedRPM = 0.0;        // For RPM-based control
    private double calculatedHoodPosition = 0.0;
    private long lastParameterUpdate = 0;
    private boolean rpmControlEnabled = Constants.AutoShootingConfig.USE_RPM_CONTROL;

    public Shooter(HardwareMap hardwareMap) {
        this.shooterMotor = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.SHOOTER_MOTOR);
        this.hoodServo = hardwareMap.get(Servo.class, Constants.HardwareConfig.HOOD_SERVO);
        this.isRunning = false;
        this.isIdle = false;


        // Set initial hood position
        setHoodPosition(hoodPositions.get(currentHoodPositionIndex));
        
        // Configure motor for velocity control if RPM mode is enabled
        if (rpmControlEnabled) {
            setupVelocityControl();
        }

        // If the shooter motor spins in the wrong direction, you can reverse it by uncommenting the next line.
        // this.shooterMotor.setDirection(DcMotor.Direction.REVERSE);
    }
    
    /**
     * Sets up the motor for velocity (RPM) control with PID gains.
     */
    private void setupVelocityControl() {
        // Reset encoder for velocity measurements
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set velocity PID coefficients from constants
        PIDFCoefficients pidCoeffs = new PIDFCoefficients(
            Constants.AutoShootingConfig.VELOCITY_KP,
            Constants.AutoShootingConfig.VELOCITY_KI,
            Constants.AutoShootingConfig.VELOCITY_KD,
            Constants.AutoShootingConfig.VELOCITY_KF
        );
        
        // Apply PID coefficients (RUN_USING_ENCODER mode for velocity control)
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoeffs);
        
        // Set zero power behavior to float for smoother control
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // =================================================================================
    // STATE MANAGEMENT METHODS
    // =================================================================================
    
    /**
     * Sets the shooter state and configures the system accordingly.
     * @param newState The new state for the shooter system
     */
    public void setState(ShooterState newState) {
        if (currentState != newState) {
            currentState = newState;
            // Additional state-specific setup can be added here if needed
        }
    }
    
    /**
     * Gets the current shooter state.
     * @return The current ShooterState
     */
    public ShooterState getState() {
        return currentState;
    }
    
    // =================================================================================
    // AUTOMATIC SHOOTING METHODS
    // =================================================================================
    
    /**
     * Sets the target distance for automatic shooting calculations.
     * @param distance The distance to target in inches
     */
    public void setTargetDistance(double distance) {
        this.targetDistance = distance;
        updateShootingParameters();
    }
    
    /**
     * Updates shooting parameters based on the current target distance.
     * Uses lookup tables with interpolation for smooth parameter changes.
     * Calculates both RPM and power values for flexibility.
     */
    private void updateShootingParameters() {
        if (System.currentTimeMillis() - lastParameterUpdate < Constants.AutoShootingConfig.PARAMETER_UPDATE_DELAY_MS) {
            return; // Don't update too frequently
        }
        
        // Clamp distance to valid range
        double clampedDistance = Math.max(Constants.AutoShootingConfig.MIN_SHOOTING_DISTANCE,
                Math.min(Constants.AutoShootingConfig.MAX_SHOOTING_DISTANCE, targetDistance));
        
        // Calculate shooting parameters using lookup tables
        if (rpmControlEnabled) {
            calculatedRPM = calculateShooterRPM(clampedDistance);
        } else {
            calculatedPower = calculateShooterPower(clampedDistance);
        }
        calculatedHoodPosition = calculateHoodPosition(clampedDistance);
        
        lastParameterUpdate = System.currentTimeMillis();
    }
    
    /**
     * Calculates the appropriate shooter power based on distance (fallback for non-RPM control).
     * @param distance Distance to target in inches
     * @return Shooter power value between 0.0 and 1.0
     */
    private double calculateShooterPower(double distance) {
        // This method is kept for backward compatibility when RPM control is disabled
        // You can still populate SHOOTER_POWER_VALUES in Constants if needed
        return 0.8; // Fallback power value
    }
    
    /**
     * Calculates the appropriate shooter RPM based on distance.
     * @param distance Distance to target in inches
     * @return Shooter RPM value
     */
    private double calculateShooterRPM(double distance) {
        if (Constants.AutoShootingConfig.ENABLE_INTERPOLATION) {
            return interpolateValue(distance,
                Constants.AutoShootingConfig.DISTANCE_BREAKPOINTS,
                Constants.AutoShootingConfig.SHOOTER_RPM_VALUES);
        } else {
            return lookupValue(distance,
                Constants.AutoShootingConfig.DISTANCE_BREAKPOINTS,
                Constants.AutoShootingConfig.SHOOTER_RPM_VALUES);
        }
    }
    
    /**
     * Calculates the appropriate hood position based on distance.
     * @param distance Distance to target in inches
     * @return Hood position value between 0.0 and 1.0
     */
    private double calculateHoodPosition(double distance) {
        if (Constants.AutoShootingConfig.ENABLE_INTERPOLATION) {
            return interpolateValue(distance,
                Constants.AutoShootingConfig.DISTANCE_BREAKPOINTS,
                Constants.AutoShootingConfig.HOOD_POSITION_VALUES);
        } else {
            return lookupValue(distance,
                Constants.AutoShootingConfig.DISTANCE_BREAKPOINTS,
                Constants.AutoShootingConfig.HOOD_POSITION_VALUES);
        }
    }
    
    /**
     * Performs linear interpolation between lookup table values.
     * @param distance The input distance
     * @param distances Array of distance breakpoints
     * @param values Array of corresponding values
     * @return Interpolated value
     */
    private double interpolateValue(double distance, double[] distances, double[] values) {
        // Handle edge cases
        if (distance <= distances[0]) return values[0];
        if (distance >= distances[distances.length - 1]) return values[values.length - 1];
        
        // Find the two breakpoints that surround our distance
        for (int i = 0; i < distances.length - 1; i++) {
            if (distance >= distances[i] && distance <= distances[i + 1]) {
                // Linear interpolation between the two points
                double t = (distance - distances[i]) / (distances[i + 1] - distances[i]);
                return values[i] + t * (values[i + 1] - values[i]);
            }
        }
        
        return values[values.length - 1]; // Fallback
    }
    
    /**
     * Performs simple lookup without interpolation (step function).
     * @param distance The input distance
     * @param distances Array of distance breakpoints
     * @param values Array of corresponding values
     * @return Lookup value
     */
    private double lookupValue(double distance, double[] distances, double[] values) {
        for (int i = distances.length - 1; i >= 0; i--) {
            if (distance >= distances[i]) {
                return values[i];
            }
        }
        return values[0]; // Fallback for distances below minimum
    }
    
    /**
     * Update method for automatic shooting mode.
     * Call this when you want the shooter to use calculated parameters.
     * Uses RPM control for better consistency when enabled.
     */
    public void updateAutomatic() {
        if (currentState == ShooterState.AUTOMATIC) {
            // Apply the calculated hood position
            double safeHoodPosition = Math.max(Constants.AutoShootingConfig.MIN_HOOD_POSITION,
                    Math.min(Constants.AutoShootingConfig.MAX_HOOD_POSITION, calculatedHoodPosition));
            setHoodPosition(safeHoodPosition);
            
            // Apply shooter speed based on control mode
            if (rpmControlEnabled) {
                // Use RPM control for better consistency
                double safeRPM = Math.max(Constants.AutoShootingConfig.MIN_SHOOTER_RPM,
                        Math.min(Constants.AutoShootingConfig.MAX_SHOOTER_RPM, calculatedRPM));
                setRPM(safeRPM);
            } else {
                // Fallback to power control
                double safePower = Math.max(Constants.AutoShootingConfig.MIN_SHOOTER_POWER,
                        Math.min(Constants.AutoShootingConfig.MAX_SHOOTER_POWER, calculatedPower));
                setPower(safePower);
            }
        }
    }
    
    // =================================================================================
    // RPM CONTROL METHODS
    // =================================================================================
    
    /**
     * Sets the shooter motor to a specific RPM using velocity control.
     * @param targetRPM The desired RPM
     */
    public void setRPM(double targetRPM) {
        if (rpmControlEnabled) {
            // Convert RPM to ticks per second for setVelocity()
            double ticksPerSecond = rpmToTicksPerSecond(targetRPM);
            shooterMotor.setVelocity(ticksPerSecond);
            isRunning = targetRPM > 0;
        } else {
            // If RPM control is disabled, fall back to power control
            // This is a rough approximation - you may want to create a better mapping
            double approximatePower = Math.min(1.0, targetRPM / 5000.0); // Assume max RPM of 5000
            setPower(approximatePower);
        }
    }
    
    /**
     * Gets the current RPM of the shooter motor.
     * @return The current RPM, or -1 if RPM control is not enabled
     */
    public double getCurrentRPM() {
        if (rpmControlEnabled) {
            double currentVelocity = shooterMotor.getVelocity(); // ticks per second
            return ticksPerSecondToRPM(currentVelocity);
        } else {
            return -1; // RPM not available in power control mode
        }
    }
    
    /**
     * Converts RPM to motor ticks per second.
     * @param rpm The RPM value
     * @return Ticks per second
     */
    private double rpmToTicksPerSecond(double rpm) {
        return (rpm / 60.0) * Constants.AutoShootingConfig.SHOOTER_MOTOR_TICKS_PER_REV;
    }
    
    /**
     * Converts motor ticks per second to RPM.
     * @param ticksPerSecond The velocity in ticks per second
     * @return RPM value
     */
    private double ticksPerSecondToRPM(double ticksPerSecond) {
        return (ticksPerSecond * 60.0) / Constants.AutoShootingConfig.SHOOTER_MOTOR_TICKS_PER_REV;
    }
    
    /**
     * Checks if the shooter RPM is within a tolerance of the target.
     * @param targetRPM The target RPM
     * @param tolerance The acceptable RPM tolerance (default: 50 RPM)
     * @return true if the current RPM is close to the target
     */
    public boolean isRPMStable(double targetRPM, double tolerance) {
        if (!rpmControlEnabled) {
            return true; // In power mode, assume it's always "stable"
        }
        
        double currentRPM = getCurrentRPM();
        return Math.abs(currentRPM - targetRPM) <= tolerance;
    }
    
    /**
     * Overloaded method with default tolerance of 50 RPM.
     */
    public boolean isRPMStable(double targetRPM) {
        return isRPMStable(targetRPM, 50.0);
    }
    
    // =================================================================================
    // MANUAL CONTROL METHODS
    // =================================================================================
    
    /**
     * Manual update method for gamepad control.
     * Works regardless of shooter state for backward compatibility.
     * @param gamepad The gamepad for manual control
     */
    public void manualUpdate(Gamepad gamepad) {
        // --- Shooter Motor Control ---
        if (gamepad.cross && isInitialized)
            setPower(Constants.ShooterConfig.SHOOTER_SPEED);

        boolean currentTriangleButtonState = gamepad.triangle;
        if(currentTriangleButtonState && !previousTriangleButtonState) {
            isInitialized = !isInitialized;

            if (isInitialized) {
                setPower(0.5);
            } else {
                setPower(0);
            }
        }
        previousTriangleButtonState = currentTriangleButtonState;

        // --- Hood Control ---
        boolean currentDpadUpState = gamepad.dpad_up;
        boolean currentDpadDownState = gamepad.dpad_down;

        if (currentDpadUpState && !previousDpadUpState) {
            // Move to the next hood position
            currentHoodPositionIndex = Math.min(hoodPositions.size() - 1, currentHoodPositionIndex + 1);
            setHoodPosition(hoodPositions.get(currentHoodPositionIndex));
        } else if (currentDpadDownState && !previousDpadDownState) {
            // Move to the previous hood position
            currentHoodPositionIndex = Math.max(0, currentHoodPositionIndex - 1);
            setHoodPosition(hoodPositions.get(currentHoodPositionIndex));
        }

        previousDpadUpState = currentDpadUpState;
        previousDpadDownState = currentDpadDownState;
    }
    public void initializeIdleSpeed(){
        if(!isIdle) {
            setPower(Constants.ShooterConfig.SHOOTER_SPEED_IDLE);
            isIdle = true;
        }
    }
    /**
     * Legacy update method for backward compatibility.
     * Simply calls manualUpdate() to maintain existing functionality.
     * @param gamepad The gamepad for manual control
     */
    public void update(Gamepad gamepad) {
        manualUpdate(gamepad);
    }
    
    // =================================================================================
    // UTILITY AND GETTER METHODS
    // =================================================================================

    public void setPower(double power) {
        shooterMotor.setPower(power);
        isRunning = power > 0;
    }

    public void setHoodPosition(double position) {
        hoodServo.setPosition(position);
    }

    /**
     * Returns the current power of the shooter motor. Useful for telemetry.
     * @return The current power of the shooter motor.
     */
    public double getMotorPower() {
        return shooterMotor.getPower();
    }

    /**
     * Returns the current position of the hood servo. Useful for telemetry.
     * @return The current position of the hood servo.
     */
    public double getServoPosition() {
        return hoodServo.getPosition();
    }

    public boolean isRunning() {
        return isRunning;
    }
    
    /**
     * Gets the current target distance for automatic shooting.
     * @return The target distance in inches
     */
    public double getTargetDistance() {
        return targetDistance;
    }
    
    /**
     * Gets the calculated shooter power for the current distance.
     * @return The calculated power value
     */
    public double getCalculatedPower() {
        return calculatedPower;
    }
    
    /**
     * Gets the calculated hood position for the current distance.
     * @return The calculated hood position value
     */
    public double getCalculatedHoodPosition() {
        return calculatedHoodPosition;
    }
    
    /**
     * Gets the calculated shooter RPM for the current distance.
     * @return The calculated RPM value, or -1 if RPM control is not enabled
     */
    public double getCalculatedRPM() {
        return calculatedRPM;
    }
    
    /**
     * Checks if RPM control is enabled.
     * @return true if using RPM control, false if using power control
     */
    public boolean isRPMControlEnabled() {
        return rpmControlEnabled;
    }
    
    /**
     * Sets whether to use RPM control or power control.
     * @param enabled true to enable RPM control, false for power control
     */
    public void setRPMControlEnabled(boolean enabled) {
        if (rpmControlEnabled != enabled) {
            rpmControlEnabled = enabled;
            if (enabled) {
                setupVelocityControl();
            }
        }
    }
    
    /**
     * Checks if the automatic shooting parameters are ready.
     * @return true if distance has been set and parameters calculated
     */
    public boolean isAutoShootingReady() {
        if (rpmControlEnabled) {
            return targetDistance > 0 && calculatedRPM > 0 && calculatedHoodPosition > 0;
        } else {
            return targetDistance > 0 && calculatedPower > 0 && calculatedHoodPosition > 0;
        }
    }
    
    /**
     * Gets a formatted string with current shooting parameters for telemetry.
     * @return Formatted parameter string
     */
    public String getShootingParametersString() {
        if (rpmControlEnabled) {
            return String.format("Dist: %.1f\", RPM: %.0f, Hood: %.2f", 
                    targetDistance, calculatedRPM, calculatedHoodPosition);
        } else {
            return String.format("Dist: %.1f\", Power: %.2f, Hood: %.2f", 
                    targetDistance, calculatedPower, calculatedHoodPosition);
        }
    }
    
    /**
     * Gets a comprehensive status string including current and target values.
     * @return Formatted status string for telemetry
     */
    public String getDetailedStatusString() {
        StringBuilder status = new StringBuilder();
        status.append(String.format("Mode: %s | ", rpmControlEnabled ? "RPM" : "Power"));
        
        if (rpmControlEnabled) {
            double currentRPM = getCurrentRPM();
            status.append(String.format("RPM: %.0f → %.0f | ", currentRPM, calculatedRPM));
            if (calculatedRPM > 0) {
                status.append(String.format("Stable: %s | ", isRPMStable(calculatedRPM) ? "✅" : "❌"));
            }
        } else {
            status.append(String.format("Power: %.2f | ", getMotorPower()));
        }
        
        status.append(String.format("Hood: %.2f", calculatedHoodPosition));
        return status.toString();
    }
}
