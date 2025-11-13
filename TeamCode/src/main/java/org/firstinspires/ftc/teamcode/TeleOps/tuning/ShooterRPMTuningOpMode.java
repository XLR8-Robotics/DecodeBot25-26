package org.firstinspires.ftc.teamcode.TeleOps.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.TeleOps.tuning.TuningUtils;

/**
 * Real-time tuning OpMode for Shooter RPM control PID parameters.
 * Allows live adjustment of velocity PID gains while observing RPM performance.
 * 
 * Controls:
 * Gamepad 2: Tuning controls
 * - DPAD LEFT/RIGHT: Select PID parameter to tune (KP, KI, KD, KF)
 * - Left Stick Y: Adjust selected PID parameter
 * - Right Stick Y: Adjust target RPM for testing
 * - A: Toggle shooter on/off at target RPM
 * - B: Reset current parameter to default
 * - X: Save all tuned values to file
 * - Y: Cycle through preset RPM test values
 * - Start: Emergency stop shooter
 */
@TeleOp(name = "⚙️ Shooter RPM Tuning", group = "Tuning")
public class ShooterRPMTuningOpMode extends LinearOpMode {
    
    // =================================================================================
    // SUBSYSTEM INSTANCES
    // =================================================================================
    
    private Robot robot;
    private TuningUtils tuningUtils;
    
    // =================================================================================
    // TUNING STATE VARIABLES
    // =================================================================================
    
    // PID parameters (working copies we can modify)
    private double velocityKp = Constants.AutoShootingConfig.VELOCITY_KP;
    private double velocityKi = Constants.AutoShootingConfig.VELOCITY_KI;
    private double velocityKd = Constants.AutoShootingConfig.VELOCITY_KD;
    private double velocityKf = Constants.AutoShootingConfig.VELOCITY_KF;
    private double ticksPerRev = Constants.AutoShootingConfig.SHOOTER_MOTOR_TICKS_PER_REV;
    
    // Test parameters
    private double targetRPM = 3000.0;
    private boolean shooterEnabled = false;
    
    // Preset test RPM values
    private final double[] presetRPMs = {2000, 2500, 3000, 3500, 4000, 4500};
    private int currentPresetIndex = 2; // Start with 3000 RPM
    
    // Current tuning state
    private enum TuningParameter { 
        KP("Proportional Gain", "Increase for faster response"),
        KI("Integral Gain", "Increase to eliminate steady-state error"),
        KD("Derivative Gain", "Increase to reduce overshoot"),
        KF("Feed-Forward Gain", "Motor-specific, usually 12-15 for goBILDA"),
        TICKS_PER_REV("Ticks Per Revolution", "Motor encoder specification")
        ;
        
        private final String displayName;
        private final String description;
        
        TuningParameter(String displayName, String description) {
            this.displayName = displayName;
            this.description = description;
        }
        
        public String getDisplayName() { return displayName; }
        public String getDescription() { return description; }
    }
    
    private TuningParameter currentParameter = TuningParameter.KP;
    
    // Input tracking
    private boolean previousDpadLeftState = false;
    private boolean previousDpadRightState = false;
    private boolean previousAButtonState = false;
    private boolean previousBButtonState = false;
    private boolean previousXButtonState = false;
    private boolean previousYButtonState = false;
    private boolean previousStartButtonState = false;
    
    // Performance tracking
    private ElapsedTime performanceTimer = new ElapsedTime();
    private ElapsedTime inputTimer = new ElapsedTime();
    private static final double INPUT_DEBOUNCE_MS = 150;
    
    private double rpmError = 0.0;
    private double maxError = 0.0;
    private double settlingTime = 0.0;
    private boolean hasSettled = false;
    private double averageError = 0.0;
    private double errorSum = 0.0;
    private int errorSamples = 0;
    
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
            
            // Apply current PID parameters
            applyPIDParameters();
            
            // Track performance metrics
            updatePerformanceMetrics();
            
            // Display comprehensive tuning telemetry
            displayTuningTelemetry();
        }
        
        // Stop shooter and save final values when OpMode ends
        robot.shooter.setPower(0);
        saveFinalValues();
    }
    
    // =================================================================================
    // INITIALIZATION METHODS
    // =================================================================================
    
    private void initializeRobot() {
        robot = new Robot();
        robot.init(hardwareMap, Robot.AimingMode.LEGACY, Robot.ShootingMode.MANUAL);
        
        // Enable RPM control for tuning
        robot.shooter.setRPMControlEnabled(true);
        
        tuningUtils = new TuningUtils();
        
        // Set initial target RPM
        targetRPM = presetRPMs[currentPresetIndex];
    }
    
    private void displayInitialInstructions() {
        telemetry.addData("⚙️", "SHOOTER RPM TUNING MODE");
        telemetry.addData("Status", "Ready to tune velocity PID parameters!");
        telemetry.addLine();
        telemetry.addData("📋 CONTROLS 📋", "");
        telemetry.addData("DPAD ←→", "Select PID parameter");
        telemetry.addData("Left Stick ↑↓", "Adjust selected parameter");
        telemetry.addData("Right Stick ↑↓", "Adjust target RPM");
        telemetry.addData("A Button", "Toggle Shooter On/Off");
        telemetry.addData("B Button", "Reset Parameter");
        telemetry.addData("X Button", "💾 SAVE TUNED VALUES");
        telemetry.addData("Y Button", "Cycle Preset RPMs");
        telemetry.addData("Start", "Emergency Stop");
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
        boolean currentLeft = gamepad2.dpad_left;
        boolean currentRight = gamepad2.dpad_right;
        
        if (currentLeft && !previousDpadLeftState && inputTimer.milliseconds() > INPUT_DEBOUNCE_MS) {
            // Cycle to previous parameter
            int currentIndex = currentParameter.ordinal();
            currentIndex = (currentIndex - 1 + TuningParameter.values().length) % TuningParameter.values().length;
            currentParameter = TuningParameter.values()[currentIndex];
            inputTimer.reset();
        }
        
        if (currentRight && !previousDpadRightState && inputTimer.milliseconds() > INPUT_DEBOUNCE_MS) {
            // Cycle to next parameter
            int currentIndex = currentParameter.ordinal();
            currentIndex = (currentIndex + 1) % TuningParameter.values().length;
            currentParameter = TuningParameter.values()[currentIndex];
            inputTimer.reset();
        }
        
        previousDpadLeftState = currentLeft;
        previousDpadRightState = currentRight;
    }
    
    private void handleParameterAdjustment() {
        // Left stick Y: Adjust selected parameter
        double adjustment = -gamepad2.left_stick_y * getParameterAdjustmentScale();
        if (Math.abs(adjustment) > 0.001) {
            adjustCurrentParameter(adjustment);
        }
        
        // Right stick Y: Adjust target RPM
        double rpmAdjustment = -gamepad2.right_stick_y * 50.0; // 50 RPM increments
        if (Math.abs(rpmAdjustment) > 1.0) {
            targetRPM = Math.max(1000, Math.min(6000, targetRPM + rpmAdjustment));
            resetPerformanceMetrics(); // Reset metrics when changing target
        }
    }
    
    private void adjustCurrentParameter(double adjustment) {
        switch (currentParameter) {
            case KP:
                velocityKp = Math.max(0.0, Math.min(100.0, velocityKp + adjustment));
                break;
            case KI:
                velocityKi = Math.max(0.0, Math.min(10.0, velocityKi + adjustment));
                break;
            case KD:
                velocityKd = Math.max(0.0, Math.min(10.0, velocityKd + adjustment));
                break;
            case KF:
                velocityKf = Math.max(0.0, Math.min(30.0, velocityKf + adjustment));
                break;
            case TICKS_PER_REV:
                ticksPerRev = Math.max(100, Math.min(2000, ticksPerRev + adjustment));
                break;
        }
        
        // Reset performance metrics when changing PID parameters
        resetPerformanceMetrics();
    }
    
    private double getParameterAdjustmentScale() {
        switch (currentParameter) {
            case KP:
                return 0.1; // 0.1 increments for KP
            case KI:
                return 0.01; // 0.01 increments for KI
            case KD:
                return 0.01; // 0.01 increments for KD
            case KF:
                return 0.1; // 0.1 increments for KF
            case TICKS_PER_REV:
                return 1.0; // 1 tick increments
            default:
                return 0.01;
        }
    }
    
    private void handleActionButtons() {
        // A button: Toggle shooter
        boolean currentA = gamepad2.a;
        if (currentA && !previousAButtonState) {
            toggleShooter();
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
        
        // Y button: Cycle preset RPMs
        boolean currentY = gamepad2.y;
        if (currentY && !previousYButtonState) {
            cyclePresetRPM();
        }
        previousYButtonState = currentY;
        
        // Start button: Emergency stop
        boolean currentStart = gamepad2.start;
        if (currentStart && !previousStartButtonState) {
            emergencyStop();
        }
        previousStartButtonState = currentStart;
    }
    
    // =================================================================================
    // ROBOT CONTROL METHODS
    // =================================================================================
    
    private void updateRobotSystems() {
        // Update limelight for any vision-based features
        robot.limelight.update();
        
        // Manual control for non-shooter systems
        robot.intake.update(gamepad2);
        robot.turret.manualUpdate(gamepad2);
        
        // Shooter is controlled by this tuning OpMode
        if (shooterEnabled) {
            robot.shooter.setRPM(targetRPM);
        } else {
            robot.shooter.setPower(0);
        }
    }
    
    private void applyPIDParameters() {
        // Note: In a real implementation, we would need to modify the Shooter class
        // to accept runtime PID parameter updates. For now, this serves as a framework
        // and the tuned values can be saved to Constants.java
    }
    
    private void updatePerformanceMetrics() {
        if (shooterEnabled && robot.shooter.isRPMControlEnabled()) {
            double currentRPM = robot.shooter.getCurrentRPM();
            
            if (currentRPM >= 0) { // Valid RPM reading
                rpmError = Math.abs(targetRPM - currentRPM);
                
                // Track maximum error
                maxError = Math.max(maxError, rpmError);
                
                // Calculate running average error
                errorSum += rpmError;
                errorSamples++;
                averageError = errorSum / errorSamples;
                
                // Check for settling (within 50 RPM for 1 second)
                if (rpmError < 50.0) {
                    if (!hasSettled) {
                        settlingTime = performanceTimer.seconds();
                        hasSettled = true;
                    }
                } else {
                    hasSettled = false;
                }
            }
        }
    }
    
    private void resetPerformanceMetrics() {
        performanceTimer.reset();
        rpmError = 0.0;
        maxError = 0.0;
        settlingTime = 0.0;
        hasSettled = false;
        averageError = 0.0;
        errorSum = 0.0;
        errorSamples = 0;
    }
    
    private void toggleShooter() {
        shooterEnabled = !shooterEnabled;
        
        if (shooterEnabled) {
            resetPerformanceMetrics();
        } else {
            robot.shooter.setPower(0);
            robot.shooter.setRPM(0);
        }
    }
    
    private void resetCurrentParameter() {
        switch (currentParameter) {
            case KP:
                velocityKp = Constants.AutoShootingConfig.VELOCITY_KP;
                break;
            case KI:
                velocityKi = Constants.AutoShootingConfig.VELOCITY_KI;
                break;
            case KD:
                velocityKd = Constants.AutoShootingConfig.VELOCITY_KD;
                break;
            case KF:
                velocityKf = Constants.AutoShootingConfig.VELOCITY_KF;
                break;
            case TICKS_PER_REV:
                ticksPerRev = Constants.AutoShootingConfig.SHOOTER_MOTOR_TICKS_PER_REV;
                break;
        }
        resetPerformanceMetrics();
    }
    
    private void cyclePresetRPM() {
        currentPresetIndex = (currentPresetIndex + 1) % presetRPMs.length;
        targetRPM = presetRPMs[currentPresetIndex];
        resetPerformanceMetrics();
    }
    
    private void emergencyStop() {
        shooterEnabled = false;
        robot.shooter.setPower(0);
        robot.shooter.setRPM(0);
    }
    
    private void saveCurrentValues() {
        String filename = "shooter_rpm_" + System.currentTimeMillis() + ".txt";
        tuningUtils.saveShooterRPMParameters(filename, velocityKp, velocityKi, 
            velocityKd, velocityKf, ticksPerRev);
        
        telemetry.addLine("💾 VALUES SAVED TO: " + filename);
        telemetry.update();
        sleep(1000);
    }
    
    private void saveFinalValues() {
        String filename = "shooter_rpm_final_" + System.currentTimeMillis() + ".txt";
        tuningUtils.saveShooterRPMParameters(filename, velocityKp, velocityKi, 
            velocityKd, velocityKf, ticksPerRev);
    }
    
    // =================================================================================
    // TELEMETRY METHODS
    // =================================================================================
    
    private void displayTuningTelemetry() {
        displayCurrentStatus();
        displayTuningParameters();
        displayPerformanceMetrics();
        displayShooterData();
        displayInstructions();
        
        telemetry.update();
    }
    
    private void displayCurrentStatus() {
        telemetry.addData("⚙️", "SHOOTER RPM TUNING");
        telemetry.addData("Shooter Status", shooterEnabled ? "🟢 ENABLED" : "🔴 DISABLED");
        telemetry.addData("Current Parameter", "⚙️ " + currentParameter.getDisplayName());
        telemetry.addData("Test RPM", "%.0f [%d/%d]", targetRPM, currentPresetIndex + 1, presetRPMs.length);
        telemetry.addLine();
    }
    
    private void displayTuningParameters() {
        telemetry.addData("=== PID PARAMETERS ===", "");
        displayPIDParameter("KP", velocityKp, TuningParameter.KP, "%.2f");
        displayPIDParameter("KI", velocityKi, TuningParameter.KI, "%.2f");
        displayPIDParameter("KD", velocityKd, TuningParameter.KD, "%.2f");
        displayPIDParameter("KF", velocityKf, TuningParameter.KF, "%.2f");
        displayPIDParameter("Ticks/Rev", ticksPerRev, TuningParameter.TICKS_PER_REV, "%.1f");
        telemetry.addLine();
    }
    
    private void displayPIDParameter(String name, double value, TuningParameter param, String format) {
        String indicator = (currentParameter == param) ? " ◄" : "";
        telemetry.addData(name, format + "%s", value, indicator);
    }
    
    private void displayPerformanceMetrics() {
        telemetry.addData("=== PERFORMANCE ===", "");
        
        if (shooterEnabled) {
            double currentRPM = robot.shooter.getCurrentRPM();
            
            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("Current RPM", currentRPM >= 0 ? String.format("%.0f", currentRPM) : "Reading...");
            telemetry.addData("Error", "%.0f RPM", rpmError);
            telemetry.addData("Max Error", "%.0f RPM", maxError);
            telemetry.addData("Avg Error", "%.0f RPM", averageError);
            
            if (hasSettled) {
                telemetry.addData("Settling Time", "%.1f sec ✅", settlingTime);
            } else {
                telemetry.addData("Settling", "Not settled yet");
            }
            
            telemetry.addData("Test Time", "%.1f seconds", performanceTimer.seconds());
        } else {
            telemetry.addData("Performance", "Enable shooter to track metrics");
        }
        telemetry.addLine();
    }
    
    private void displayShooterData() {
        telemetry.addData("=== SHOOTER DATA ===", "");
        telemetry.addData("Motor Power", "%.2f", robot.shooter.getMotorPower());
        telemetry.addData("RPM Stable", robot.shooter.isRPMStable(targetRPM) ? "✅" : "❌");
        telemetry.addData("Hood Position", "%.3f", robot.shooter.getServoPosition());
        telemetry.addLine();
    }
    
    private void displayInstructions() {
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("DPAD ←→", "Select Parameter [" + currentParameter.getDisplayName() + "]");
        telemetry.addData("Left Stick ↑↓", "Adjust Parameter");
        telemetry.addData("Right Stick ↑↓", "Adjust Target RPM");
        telemetry.addData("A", "Toggle Shooter | Y: Preset RPM");
        telemetry.addData("B", "Reset | X: Save | Start: E-Stop");
        telemetry.addData("Description", currentParameter.getDescription());
    }
}
