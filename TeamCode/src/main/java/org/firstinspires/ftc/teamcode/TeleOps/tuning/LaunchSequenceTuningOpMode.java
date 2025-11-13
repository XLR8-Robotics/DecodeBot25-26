package org.firstinspires.ftc.teamcode.TeleOps.tuning;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.constants.MecanumConstants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSequenceController;
import org.firstinspires.ftc.teamcode.TeleOps.tuning.TuningUtils;

/**
 * Real-time tuning OpMode for launch sequence timing parameters.
 * Allows live adjustment of timing values while observing sequence behavior.
 * 
 * Controls:
 * Gamepad 1: Drive robot for positioning tests
 * Gamepad 2: Tuning controls
 * - DPAD UP/DOWN: Select timing parameter to tune
 * - Left Stick Y: Adjust selected timing parameter
 * - A: Start launch sequence (test current timings)
 * - B: Cancel/Reset current parameter to default
 * - X: Save all tuned values to file
 * - Y: Emergency stop sequence
 * - Cross (×): Triple press to cancel (standard launch sequence control)
 */
@TeleOp(name = "🚀 Launch Sequence Tuning", group = "Tuning")
public class LaunchSequenceTuningOpMode extends LinearOpMode {
    
    // =================================================================================
    // SUBSYSTEM INSTANCES
    // =================================================================================
    
    private Robot robot;
    private Follower follower;
    private TuningUtils tuningUtils;
    
    // =================================================================================
    // TUNING STATE VARIABLES
    // =================================================================================
    
    // Timing parameters (working copies we can modify)
    private long shooterSpinUpTime = Constants.LaunchSequenceConfig.SHOOTER_SPIN_UP_TIME_MS;
    private long intakeReverseTime = Constants.LaunchSequenceConfig.INTAKE_REVERSE_TIME_MS;
    private long liftServoHoldTime = Constants.LaunchSequenceConfig.LIFT_SERVO_HOLD_TIME_MS;
    private long singlePressTimeout = Constants.LaunchSequenceConfig.SINGLE_PRESS_TIMEOUT_MS;
    private long triplePressTimeout = Constants.LaunchSequenceConfig.TRIPLE_PRESS_TIMEOUT_MS;
    
    // Current tuning state
    private enum TimingParameter { 
        SHOOTER_SPIN_UP("Shooter Spin-Up Time", "Time for shooter to reach speed"),
        INTAKE_REVERSE("Intake Reverse Time", "Time to reverse intake when cancelled"),
        LIFT_HOLD("Lift Servo Hold Time", "Time to hold lift servo position"),
        SINGLE_PRESS("Single Press Timeout", "Window for detecting single press"),
        TRIPLE_PRESS("Triple Press Timeout", "Window for detecting triple press")
        ;
        
        private final String displayName;
        private final String description;
        
        TimingParameter(String displayName, String description) {
            this.displayName = displayName;
            this.description = description;
        }
        
        public String getDisplayName() { return displayName; }
        public String getDescription() { return description; }
    }
    
    private TimingParameter currentParameter = TimingParameter.SHOOTER_SPIN_UP;
    
    // Input tracking
    private boolean previousDpadUpState = false;
    private boolean previousDpadDownState = false;
    private boolean previousAButtonState = false;
    private boolean previousBButtonState = false;
    private boolean previousXButtonState = false;
    private boolean previousYButtonState = false;
    
    // Timing and performance tracking
    private ElapsedTime inputTimer = new ElapsedTime();
    private ElapsedTime sequenceTimer = new ElapsedTime();
    private static final double INPUT_DEBOUNCE_MS = 150;
    
    // Sequence tracking for analysis
    private LaunchSequenceController.LaunchState lastSequenceState = LaunchSequenceController.LaunchState.IDLE;
    private long stateStartTime = 0;
    private StringBuilder sequenceLog = new StringBuilder();
    
    // =================================================================================
    // MAIN OPMODE METHODS
    // =================================================================================
    
    @Override
    public void runOpMode() {
        initializeRobot();
        
        displayInitialInstructions();
        waitForStart();

        while (opModeIsActive()) {
            // Update robot systems
            updateRobotSystems();
            
            // Handle tuning inputs
            handleTuningInputs();
            
            // Handle drive controls
            handleDriveControls();
            
            // Track sequence performance
            trackSequencePerformance();
            
            // Display comprehensive tuning telemetry
            displayTuningTelemetry();
        }
        
        // Save final values when OpMode ends
        saveFinalValues();
    }
    
    // =================================================================================
    // INITIALIZATION METHODS
    // =================================================================================
    
    private void initializeRobot() {
        // Initialize with enhanced systems
        follower = new Follower(hardwareMap, MecanumConstants.class);
        follower.setStartingPose(new com.pedropathing.geometry.Pose(0, 0, 0));
        
        robot = new Robot(follower);
        robot.init(hardwareMap, Robot.AimingMode.ENHANCED, Robot.ShootingMode.MANUAL);
        
        tuningUtils = new TuningUtils();
    }
    
    private void displayInitialInstructions() {
        telemetry.addData("🚀", "LAUNCH SEQUENCE TUNING MODE");
        telemetry.addData("Status", "Ready to tune timing parameters!");
        telemetry.addLine();
        telemetry.addData("📋 CONTROLS 📋", "");
        telemetry.addData("Gamepad 1", "Drive for positioning");
        telemetry.addData("DPAD ↑↓", "Select timing parameter");
        telemetry.addData("Left Stick ↑↓", "Adjust timing value");
        telemetry.addData("A Button", "🚀 Start Launch Sequence");
        telemetry.addData("B Button", "Reset Parameter");
        telemetry.addData("X Button", "💾 SAVE TUNED VALUES");
        telemetry.addData("Y Button", "Emergency Stop");
        telemetry.addData("Cross (×)", "Standard launch controls (single/triple press)");
        telemetry.update();
    }
    
    // =================================================================================
    // INPUT HANDLING METHODS
    // =================================================================================
    
    private void handleTuningInputs() {
        // Parameter selection
        handleParameterSelection();
        
        // Parameter adjustment
        handleParameterAdjustment();
        
        // Action buttons
        handleActionButtons();
    }
    
    private void handleParameterSelection() {
        boolean currentUp = gamepad2.dpad_up;
        boolean currentDown = gamepad2.dpad_down;
        
        if (currentUp && !previousDpadUpState && inputTimer.milliseconds() > INPUT_DEBOUNCE_MS) {
            // Cycle to next parameter
            int currentIndex = currentParameter.ordinal();
            currentIndex = (currentIndex + 1) % TimingParameter.values().length;
            currentParameter = TimingParameter.values()[currentIndex];
            inputTimer.reset();
        }
        
        if (currentDown && !previousDpadDownState && inputTimer.milliseconds() > INPUT_DEBOUNCE_MS) {
            // Cycle to previous parameter
            int currentIndex = currentParameter.ordinal();
            currentIndex = (currentIndex - 1 + TimingParameter.values().length) % TimingParameter.values().length;
            currentParameter = TimingParameter.values()[currentIndex];
            inputTimer.reset();
        }
        
        previousDpadUpState = currentUp;
        previousDpadDownState = currentDown;
    }
    
    private void handleParameterAdjustment() {
        // Left stick Y: Adjust selected timing parameter
        double adjustment = -gamepad2.left_stick_y * getParameterAdjustmentScale();
        if (Math.abs(adjustment) > 1.0) { // Minimum 1ms adjustment
            adjustCurrentParameter((long)adjustment);
        }
    }
    
    private void adjustCurrentParameter(long adjustment) {
        switch (currentParameter) {
            case SHOOTER_SPIN_UP:
                shooterSpinUpTime = Math.max(500, Math.min(10000, shooterSpinUpTime + adjustment));
                break;
            case INTAKE_REVERSE:
                intakeReverseTime = Math.max(200, Math.min(5000, intakeReverseTime + adjustment));
                break;
            case LIFT_HOLD:
                liftServoHoldTime = Math.max(100, Math.min(3000, liftServoHoldTime + adjustment));
                break;
            case SINGLE_PRESS:
                singlePressTimeout = Math.max(50, Math.min(1000, singlePressTimeout + adjustment));
                break;
            case TRIPLE_PRESS:
                triplePressTimeout = Math.max(100, Math.min(2000, triplePressTimeout + adjustment));
                break;
        }
    }
    
    private double getParameterAdjustmentScale() {
        switch (currentParameter) {
            case SHOOTER_SPIN_UP:
                return 50.0; // 50ms increments
            case INTAKE_REVERSE:
                return 25.0; // 25ms increments
            case LIFT_HOLD:
                return 10.0; // 10ms increments
            case SINGLE_PRESS:
            case TRIPLE_PRESS:
                return 5.0;  // 5ms increments for button timing
            default:
                return 10.0;
        }
    }
    
    private void handleActionButtons() {
        // A button: Start launch sequence
        boolean currentA = gamepad2.a;
        if (currentA && !previousAButtonState) {
            startTestSequence();
        }
        previousAButtonState = currentA;
        
        // B button: Reset current parameter
        boolean currentB = gamepad2.b;
        if (currentB && !previousBButtonState) {
            resetCurrentParameter();
        }
        previousBButtonState = currentB;
        
        // X button: Save values
        boolean currentX = gamepad2.x;
        if (currentX && !previousXButtonState) {
            saveCurrentValues();
        }
        previousXButtonState = currentX;
        
        // Y button: Emergency stop
        boolean currentY = gamepad2.y;
        if (currentY && !previousYButtonState) {
            emergencyStop();
        }
        previousYButtonState = currentY;
    }
    
    // =================================================================================
    // ROBOT CONTROL METHODS
    // =================================================================================
    
    private void updateRobotSystems() {
        if (follower != null) {
            follower.update();
        }
        
        // Update robot with launch sequence (using standard gamepad2 for cross button)
        robot.update(gamepad2);
    }
    
    private void handleDriveControls() {
        if (follower != null) {
            follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y, 
                -gamepad1.left_stick_x, 
                -gamepad1.right_stick_x);
        }
    }
    
    private void trackSequencePerformance() {
        if (robot.launchSequenceController != null) {
            LaunchSequenceController.LaunchState currentState = robot.launchSequenceController.getCurrentState();
            
            // Detect state changes and log timing
            if (currentState != lastSequenceState) {
                long currentTime = System.currentTimeMillis();
                
                if (lastSequenceState != LaunchSequenceController.LaunchState.IDLE) {
                    // Log the time spent in the previous state
                    long timeInState = currentTime - stateStartTime;
                    sequenceLog.append(String.format("%s: %dms -> %s\n", 
                        lastSequenceState.toString(), timeInState, currentState.toString()));
                }
                
                lastSequenceState = currentState;
                stateStartTime = currentTime;
                
                // Reset timer when sequence starts
                if (currentState == LaunchSequenceController.LaunchState.SPOOLING) {
                    sequenceTimer.reset();
                    sequenceLog = new StringBuilder();
                    sequenceLog.append("=== NEW SEQUENCE STARTED ===\n");
                }
            }
        }
    }
    
    private void startTestSequence() {
        // Manually start the launch sequence for testing
        if (robot.launchSequenceController != null) {
            boolean started = robot.startLaunchSequence();
            if (started) {
                sequenceTimer.reset();
                sequenceLog = new StringBuilder();
                sequenceLog.append("=== MANUAL TEST SEQUENCE ===\n");
            }
        }
    }
    
    private void resetCurrentParameter() {
        switch (currentParameter) {
            case SHOOTER_SPIN_UP:
                shooterSpinUpTime = Constants.LaunchSequenceConfig.SHOOTER_SPIN_UP_TIME_MS;
                break;
            case INTAKE_REVERSE:
                intakeReverseTime = Constants.LaunchSequenceConfig.INTAKE_REVERSE_TIME_MS;
                break;
            case LIFT_HOLD:
                liftServoHoldTime = Constants.LaunchSequenceConfig.LIFT_SERVO_HOLD_TIME_MS;
                break;
            case SINGLE_PRESS:
                singlePressTimeout = Constants.LaunchSequenceConfig.SINGLE_PRESS_TIMEOUT_MS;
                break;
            case TRIPLE_PRESS:
                triplePressTimeout = Constants.LaunchSequenceConfig.TRIPLE_PRESS_TIMEOUT_MS;
                break;
        }
    }
    
    private void emergencyStop() {
        if (robot.launchSequenceController != null) {
            robot.emergencyStopLaunchSequence();
        }
        sequenceLog.append("=== EMERGENCY STOP ===\n");
    }
    
    private void saveCurrentValues() {
        String filename = "launch_sequence_" + System.currentTimeMillis() + ".txt";
        tuningUtils.saveLaunchSequenceParameters(filename, shooterSpinUpTime, intakeReverseTime, 
            liftServoHoldTime, singlePressTimeout, triplePressTimeout);
        
        telemetry.addLine("💾 VALUES SAVED TO: " + filename);
        telemetry.update();
        sleep(1000);
    }
    
    private void saveFinalValues() {
        String filename = "launch_sequence_final_" + System.currentTimeMillis() + ".txt";
        tuningUtils.saveLaunchSequenceParameters(filename, shooterSpinUpTime, intakeReverseTime, 
            liftServoHoldTime, singlePressTimeout, triplePressTimeout);
    }
    
    // =================================================================================
    // TELEMETRY METHODS
    // =================================================================================
    
    private void displayTuningTelemetry() {
        displayCurrentStatus();
        displayTuningParameters();
        displaySequenceStatus();
        displayPerformanceLog();
        displayInstructions();
        
        telemetry.update();
    }
    
    private void displayCurrentStatus() {
        telemetry.addData("🚀", "LAUNCH SEQUENCE TUNING");
        
        if (robot.launchSequenceController != null) {
            LaunchSequenceController.LaunchState state = robot.launchSequenceController.getCurrentState();
            String stateColor = (state == LaunchSequenceController.LaunchState.IDLE) ? "🟢" : "🔴";
            telemetry.addData("Sequence State", stateColor + " " + state.toString());
            
            if (state != LaunchSequenceController.LaunchState.IDLE) {
                telemetry.addData("State Time", "%.1f seconds", robot.launchSequenceController.getStateTime());
            }
        }
        
        telemetry.addData("Current Parameter", "⚙️ " + currentParameter.getDisplayName());
        telemetry.addLine();
    }
    
    private void displayTuningParameters() {
        telemetry.addData("=== TIMING PARAMETERS ===", "");
        
        displayTimingParameter("Shooter Spin-Up", shooterSpinUpTime, TimingParameter.SHOOTER_SPIN_UP);
        displayTimingParameter("Intake Reverse", intakeReverseTime, TimingParameter.INTAKE_REVERSE);
        displayTimingParameter("Lift Hold", liftServoHoldTime, TimingParameter.LIFT_HOLD);
        displayTimingParameter("Single Press", singlePressTimeout, TimingParameter.SINGLE_PRESS);
        displayTimingParameter("Triple Press", triplePressTimeout, TimingParameter.TRIPLE_PRESS);
        
        telemetry.addLine();
    }
    
    private void displayTimingParameter(String name, long value, TimingParameter param) {
        String indicator = (currentParameter == param) ? " ◄" : "";
        telemetry.addData(name, "%.1f sec (%d ms)%s", value / 1000.0, value, indicator);
    }
    
    private void displaySequenceStatus() {
        telemetry.addData("=== SEQUENCE STATUS ===", "");
        
        if (robot.launchSequenceController != null) {
            telemetry.addData("Status", robot.launchSequenceController.getStatusString());
        }
        
        telemetry.addData("Total Sequence Time", "%.1f seconds", sequenceTimer.seconds());
        telemetry.addData("Parameter Description", currentParameter.getDescription());
        telemetry.addLine();
    }
    
    private void displayPerformanceLog() {
        telemetry.addData("=== RECENT LOG ===", "");
        
        // Display last few lines of sequence log
        String[] logLines = sequenceLog.toString().split("\n");
        int startIndex = Math.max(0, logLines.length - 4); // Show last 4 lines
        
        for (int i = startIndex; i < logLines.length; i++) {
            if (i < logLines.length && !logLines[i].trim().isEmpty()) {
                telemetry.addData("", logLines[i].trim());
            }
        }
        
        telemetry.addLine();
    }
    
    private void displayInstructions() {
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("DPAD ↑↓", "Select Parameter [" + (currentParameter.ordinal() + 1) + "/5]");
        telemetry.addData("Left Stick ↑↓", "Adjust Timing");
        telemetry.addData("A", "Test Sequence | B: Reset | X: Save");
        telemetry.addData("Y", "E-Stop | Cross: Standard Launch");
    }
}
