package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.config.Constants;

/**
 * Manages the entire launch sequence from start to finish.
 * This class is a state machine that progresses through the different
 * stages of launching a projectile.
 */
public class LaunchSequenceController {

    // =================================================================================
    // LAUNCH SEQUENCE STATES
    // =================================================================================
    
    /**
     * Defines the possible states of the launch sequence.
     */
    public enum LaunchState {
        IDLE("Idle"),
        FEEDING("Feeding projectile"),
        LIFTING("Lifting projectile"),
        FINISHING("Finishing"),
        CANCELLED("Cancelled");
        
        private final String description;
        
        LaunchState(String description) {
            this.description = description;
        }
        
        public String getDescription() {
            return description;
        }
    }
    
    // =================================================================================
    // DEPENDENCIES
    // =================================================================================
    
    private final Intake intake;
    private final Shooter shooter;
    private final Turret turret;
    private final Limelight limelight;
    final BasicDriveTrain driveTrain;
    
    // =================================================================================
    // STATE VARIABLES
    // =================================================================================
    
    private LaunchState currentState = LaunchState.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ButtonPressDetector crossButton = new ButtonPressDetector();
    
    private boolean shooterBlocked = true;
    
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
     * @return true if sequence was started, false if already running or prerequisites not met
     */
    public boolean startSequence() {
        if (currentState != LaunchState.IDLE) {
            return false; // Already running
        }

        // Check if shooter is spinning
        if (!shooter.isRunning()) {
            return false; // Don't launch if shooter isn't ready/spinning
        }

        // Check if shooter is manually disabled
        if (shooter.isShooterMotorDisabled()) {
            return false; // Don't launch if motor is manually disabled
        }

        prepareForLaunch();
        // Spooling wait time removed, proceed directly to FEEDING
        transitionToState(LaunchState.FEEDING);
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
        
        // --- 1. Launch Sequence Inputs (Cross Button) ---
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
     * Handles the FEEDING state - running intake until projectile is detected.
     */
    private void updateFeedingState() {
        intake.setPower(Constants.IntakeConfig.INTAKE_SPEED);
        // Ensure shooter is at Launch RPM
        shooter.setRPM(Constants.LaunchSequenceConfig.SHOOTER_LAUNCH_RPM);

        // Simplified logic: Run intake for a set time then lift
        if(stateTimer.milliseconds() > 1000){ // 1 second to feed
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
        setShooterBlocker(); // Block shooter
        transitionToState(LaunchState.IDLE);
    }
    
    /**
     * Handles the CANCELLED state - reversing intake to clear projectile.
     */
    private void updateCancelledState() {
        intake.setPower(-Constants.IntakeConfig.INTAKE_SPEED);
        
        if (stateTimer.milliseconds() >= Constants.LaunchSequenceConfig.INTAKE_REVERSE_TIME_MS) {
            stopAllMotors();
            setShooterBlocker(); // Block shooter
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
     * Prepares hardware for launch sequence.
     */
    private void prepareForLaunch() {
        setShooterBlocker();
    }
    
    /**
     * Stops all motors safely.
     */
    private void stopAllMotors() {
        if (driveTrain != null) {
            driveTrain.drive(0, 0, 0);
        }
        intake.setPower(0);
        turret.setPower(0);
        // Shooter is not stopped here to maintain idle speed or spin down control separately
    }
    
    /**
     * Controls the shooter blocker position.
     */
    private void setShooterBlocker() {
        if(shooterBlocked)
        {
            turret.setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_ZERO_POSITION);
        }
        else
        {
            turret.setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION);
        }

        // Blocking wait - not ideal but preserving logic
        ElapsedTime blockerTimer = new ElapsedTime();
        while(blockerTimer.milliseconds() < 150){
            // Wait
        }
        shooterBlocked = !shooterBlocked;
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
