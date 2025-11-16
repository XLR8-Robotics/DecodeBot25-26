package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.teamcode.utils.ShooterTable;

/**
 * Dedicated controller for managing the automated launch sequence.
 * Provides a clean state machine implementation with clear separation of concerns.
 */
public class LaunchSequenceController {
    
    // =================================================================================
    // STATE DEFINITIONS
    // =================================================================================
    
    /**
     * Launch sequence states with clear documentation.
     */
    public enum LaunchState {
        IDLE("Ready to start launch sequence"),
        SPOOLING("Shooter spinning up to speed"),
        FEEDING("Intake feeding projectile to shooter"),
        LIFTING("Lift servo moving projectile into position"),
        FINISHING("Sequence complete, cleaning up"),
        CANCELLED("Sequence cancelled, reversing intake");
        
        private final String description;
        
        LaunchState(String description) {
            this.description = description;
        }
        
        public String getDescription() {
            return description;
        }
    }
    
    // =================================================================================
    // SUBSYSTEM REFERENCES
    // =================================================================================
    
    private final Intake intake;
    private final Shooter shooter;
    private final Turret turret;
    private final Limelight limelight;
    private final BasicDriveTrain driveTrain;
    
    // =================================================================================
    // STATE VARIABLES
    // =================================================================================
    
    private LaunchState currentState = LaunchState.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ButtonPressDetector crossButton = new ButtonPressDetector();
    
    // =================================================================================
    // CONSTRUCTOR
    // =================================================================================
    
    /**
     * Creates a new LaunchSequenceController with references to required subsystems.
     */
    public LaunchSequenceController(Intake intake, Shooter shooter, Turret turret, 
                                   Limelight limelight, BasicDriveTrain driveTrain) {
        this.intake = intake;
        this.shooter = shooter;
        this.turret = turret;
        this.limelight = limelight;
        this.driveTrain = driveTrain;
    }
    
    // =================================================================================
    // PUBLIC CONTROL METHODS
    // =================================================================================
    
    /**
     * Main update method - call this every loop cycle.
     * Handles both input processing and state machine updates.
     */
    public void update(Gamepad gamepad) {
        handleInput(gamepad);
        updateStateMachine();
    }
    
    /**
     * Manually start the launch sequence.
     * @return true if sequence was started, false if already running
     */
    public boolean startSequence() {
        if (currentState != LaunchState.IDLE) {
            return false; // Already running
        }
        
        transitionToState(LaunchState.SPOOLING);
        //configureShooting();
        prepareForLaunch();
        return true;
    }
    
    /**
     * Cancel the current launch sequence.
     * @return true if sequence was cancelled, false if not running
     */
    public boolean cancelSequence() {
        if (currentState == LaunchState.IDLE || currentState == LaunchState.CANCELLED) {
            return false; // Not running or already cancelled
        }
        
        transitionToState(LaunchState.CANCELLED);
        return true;
    }
    
    /**
     * Emergency stop - immediately return to idle state.
     */
    public void emergencyStop() {
        stopAllMotors();
        transitionToState(LaunchState.IDLE);
    }
    
    // =================================================================================
    // INPUT HANDLING
    // =================================================================================
    
    /**
     * Handles gamepad input for launch sequence control.
     */
    private void handleInput(Gamepad gamepad) {
        ButtonPressDetector.PressType pressType = crossButton.update(gamepad.cross);
        
        switch (pressType) {
            case SINGLE_PRESS:
                handleSinglePress();
                break;
            case TRIPLE_PRESS:
                handleTriplePress();
                break;
            case NO_PRESS:
                // No action needed
                break;
        }
    }
    
    /**
     * Handles single cross button press - starts sequence if idle.
     */
    private void handleSinglePress() {
        if (currentState == LaunchState.IDLE) {
            startSequence();
        }
    }
    
    /**
     * Handles triple cross button press - cancels sequence if running.
     */
    private void handleTriplePress() {
        if (currentState != LaunchState.IDLE) {
            cancelSequence();
        }
    }
    
    // =================================================================================
    // STATE MACHINE IMPLEMENTATION
    // =================================================================================
    
    /**
     * Updates the state machine based on the current state.
     */
    private void updateStateMachine() {
        switch (currentState) {
            case IDLE:
                // Nothing to do - waiting for input
                break;
            case SPOOLING:
                updateSpoolingState();
                break;
            case FEEDING:
                updateFeedingState();
                break;
            case LIFTING:
                updateLiftingState();
                break;
            case FINISHING:
                updateFinishingState();
                break;
            case CANCELLED:
                updateCancelledState();
                break;
        }
    }
    
    /**
     * Handles the SPOOLING state - waiting for shooter to reach speed.
     */
    private void updateSpoolingState() {
        if (stateTimer.milliseconds() >= Constants.LaunchSequenceConfig.SHOOTER_SPIN_UP_TIME_MS) {
            transitionToState(LaunchState.FEEDING);
        }
    }
    
    /**
     * Handles the FEEDING state - running intake until projectile is detected.
     */
    private void updateFeedingState() {
        intake.setPower(Constants.IntakeConfig.INTAKE_SPEED);
        
        if (!intake.isObjectDetected()) {
            transitionToState(LaunchState.LIFTING);
        }
    }
    
    /**
     * Handles the LIFTING state - lifting projectile into shooter.
     */
    private void updateLiftingState() {
        intake.setLiftPosition(Constants.IntakeConfig.LIFT_SERVO_LIFTING_POSITION);
        
        if (stateTimer.milliseconds() >= Constants.LaunchSequenceConfig.LIFT_SERVO_HOLD_TIME_MS) {
            intake.setLiftPosition(Constants.IntakeConfig.LIFT_SERVO_NOT_LIFTING_POSITION);
            transitionToState(LaunchState.FINISHING);
        }
    }
    
    /**
     * Handles the FINISHING state - cleanup after successful launch.
     */
    private void updateFinishingState() {
        stopAllMotors();
        setShooterBlocker(true); // Block shooter
        transitionToState(LaunchState.IDLE);
    }
    
    /**
     * Handles the CANCELLED state - reversing intake to clear projectile.
     */
    private void updateCancelledState() {
        intake.setPower(-Constants.IntakeConfig.INTAKE_SPEED);
        
        if (stateTimer.milliseconds() >= Constants.LaunchSequenceConfig.INTAKE_REVERSE_TIME_MS) {
            stopAllMotors();
            setShooterBlocker(true); // Block shooter
            transitionToState(LaunchState.IDLE);
        }
    }
    
    // =================================================================================
    // STATE TRANSITION HELPERS
    // =================================================================================
    
    /**
     * Transitions to a new state and resets the timer.
     */
    private void transitionToState(LaunchState newState) {
        currentState = newState;
        stateTimer.reset();
    }
    
    /**
     * Configures shooter parameters based on current limelight data.
     */
    private void configureShooting() {
        if (limelight.hasTarget()) {
            // Use distance-based shooting parameters
            double distance = limelight.getDistanceToTarget();
            ShooterTable.ShotParams shot = ShooterTable.getInterpolatedShot(distance);
            shooter.setPower(shot.power);
            shooter.setHoodPosition(shot.hood);
        } else {
            // Use default parameters
            shooter.setPower(Constants.ShooterConfig.SHOOTER_SPEED);
            shooter.setHoodPosition(Constants.ShooterConfig.HOOD_CENTER);
        }
    }
    
    /**
     * Prepares hardware for launch sequence.
     */
    private void prepareForLaunch() {
        setShooterBlocker(false); // Unblock shooter
    }
    
    /**
     * Stops all motors safely.
     */
    private void stopAllMotors() {
        if (driveTrain != null) {
            driveTrain.drive(0, 0, 0);
        }
        intake.setPower(0);
        shooter.setPower(0);
        turret.setPower(0);
    }
    
    /**
     * Controls the shooter blocker position.
     */
    private void setShooterBlocker(boolean blocked) {
        double position = blocked 
            ? Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION
            : Constants.TurretConfig.SHOOTER_BLOCKER_ZERO_POSITION;
        turret.setShooterBlockerPosition(position);
    }
    
    // =================================================================================
    // PUBLIC GETTER METHODS
    // =================================================================================
    
    /**
     * Gets the current launch sequence state.
     */
    public LaunchState getCurrentState() {
        return currentState;
    }
    
    /**
     * Gets a human-readable description of the current state.
     */
    public String getCurrentStateDescription() {
        return currentState.getDescription();
    }
    
    /**
     * Checks if the launch sequence is currently running.
     */
    public boolean isRunning() {
        return currentState != LaunchState.IDLE;
    }
    
    /**
     * Gets the time spent in the current state.
     */
    public double getStateTime() {
        return stateTimer.seconds();
    }
    
    /**
     * Gets formatted status information for telemetry.
     */
    public String getStatusString() {
        return String.format("%s (%.1fs)", 
            currentState.getDescription(), 
            getStateTime());
    }
    
    // =================================================================================
    // BUTTON PRESS DETECTION HELPER CLASS
    // =================================================================================
    
    /**
     * Helper class to detect single vs triple button presses reliably.
     */
    private static class ButtonPressDetector {
        
        public enum PressType {
            NO_PRESS,
            SINGLE_PRESS,
            TRIPLE_PRESS
        }
        
        private boolean previousButtonState = false;
        private int pressCount = 0;
        private final ElapsedTime pressTimer = new ElapsedTime();
        
        /**
         * Updates the detector with the current button state.
         * @param currentButtonState The current state of the button
         * @return The type of press detected
         */
        public PressType update(boolean currentButtonState) {
            // Detect button press (transition from false to true)
            if (currentButtonState && !previousButtonState) {
                handleButtonPress();
            }
            
            previousButtonState = currentButtonState;
            
            // Check for timeout and determine press type
            return evaluatePressType();
        }
        
        private void handleButtonPress() {
            if (pressTimer.milliseconds() > Constants.LaunchSequenceConfig.TRIPLE_PRESS_TIMEOUT_MS) {
                // Timeout exceeded, start new press sequence
                pressCount = 1;
                pressTimer.reset();
            } else {
                // Within timeout, increment count
                pressCount++;
            }
        }
        
        private PressType evaluatePressType() {
            if (pressCount == 0) {
                return PressType.NO_PRESS;
            }
            
            if (pressCount >= Constants.LaunchSequenceConfig.TRIPLE_PRESS_COUNT) {
                // Triple press detected
                pressCount = 0; // Reset for next sequence
                return PressType.TRIPLE_PRESS;
            }
            
            if (pressCount == 1 && 
                pressTimer.milliseconds() > Constants.LaunchSequenceConfig.SINGLE_PRESS_TIMEOUT_MS) {
                // Single press timeout reached
                pressCount = 0; // Reset for next sequence
                return PressType.SINGLE_PRESS;
            }
            
            return PressType.NO_PRESS; // Still waiting for more presses or timeout
        }
    }
}
