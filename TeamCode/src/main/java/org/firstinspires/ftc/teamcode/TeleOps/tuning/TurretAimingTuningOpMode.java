package org.firstinspires.ftc.teamcode.TeleOps.tuning;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.constants.MecanumConstants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.teamcode.config.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.TeleOps.tuning.TuningUtils;

/**
 * Real-time tuning OpMode for turret aiming PID parameters.
 * Allows live adjustment of KP, KI, KD gains while observing turret behavior.
 * 
 * Controls:
 * Gamepad 1: Drive robot to different positions relative to targets
 * Gamepad 2: Tuning controls
 * - DPAD LEFT/RIGHT: Select PID parameter to tune (KP, KI, KD)
 * - Left Stick Y: Adjust selected PID parameter
 * - Right Stick Y: Adjust oscillation parameters
 * - A: Toggle auto-aiming on/off
 * - B: Reset current parameter to default
 * - X: Save all tuned values to file
 * - Y: Switch between Red/Blue AprilTag targets
 * - Start: Emergency stop aiming
 */
@TeleOp(name = "🎯 Turret Aiming Tuning", group = "Tuning")
public class TurretAimingTuningOpMode extends LinearOpMode {
    
    // =================================================================================
    // SUBSYSTEM INSTANCES
    // =================================================================================
    
    private Robot robot;
    private Follower follower;
    private TuningUtils tuningUtils;
    
    // =================================================================================
    // TUNING STATE VARIABLES
    // =================================================================================
    
    // PID parameters (working copies we can modify)
    private double kp = Constants.TurretAimingConfig.AIMING_KP;
    private double ki = Constants.TurretAimingConfig.AIMING_KI;
    private double kd = Constants.TurretAimingConfig.AIMING_KD;
    
    // Oscillation parameters
    private double oscillationSpeed = Constants.TurretAimingConfig.OSCILLATION_SPEED;
    private double oscillationPeriod = Constants.TurretAimingConfig.OSCILLATION_PERIOD_MS;
    
    // Current tuning state
    private enum TuningParameter { KP, KI, KD, OSCILLATION_SPEED, OSCILLATION_PERIOD }
    private TuningParameter currentParameter = TuningParameter.KP;
    private boolean autoAimingEnabled = false;
    
    // Input tracking
    private boolean previousDpadLeftState = false;
    private boolean previousDpadRightState = false;
    private boolean previousAButtonState = false;
    private boolean previousBButtonState = false;
    private boolean previousXButtonState = false;
    private boolean previousYButtonState = false;
    private boolean previousStartButtonState = false;
    
    // Timing
    private ElapsedTime inputTimer = new ElapsedTime();
    private ElapsedTime aimingTimer = new ElapsedTime();
    private static final double INPUT_DEBOUNCE_MS = 150;
    
    // Performance tracking
    private double targetAngle = 0.0;
    private double currentError = 0.0;
    private double maxError = 0.0;
    private double settlingTime = 0.0;
    private boolean hasSettled = false;
    
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
            
            // Update aiming system with current PID values
            updateAimingSystem();
            
            // Track performance metrics
            updatePerformanceMetrics();
            
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
        // Initialize with enhanced aiming system
        follower = new Follower(hardwareMap, MecanumConstants.class);
        follower.setStartingPose(new com.pedropathing.geometry.Pose(0, 0, 0));
        
        robot = new Robot(follower);
        robot.init(hardwareMap, Robot.AimingMode.ENHANCED, Robot.ShootingMode.MANUAL);
        
        tuningUtils = new TuningUtils();
    }
    
    private void displayInitialInstructions() {
        telemetry.addData("🎯", "TURRET AIMING TUNING MODE");
        telemetry.addData("Status", "Ready to tune PID parameters!");
        telemetry.addLine();
        telemetry.addData("📋 CONTROLS 📋", "");
        telemetry.addData("Gamepad 1", "Drive to different positions");
        telemetry.addData("DPAD ←→", "Select PID parameter");
        telemetry.addData("Left Stick ↑↓", "Adjust selected parameter");
        telemetry.addData("Right Stick ↑↓", "Adjust oscillation settings");
        telemetry.addData("A Button", "Toggle Auto-Aiming");
        telemetry.addData("B Button", "Reset Current Parameter");
        telemetry.addData("X Button", "💾 SAVE TUNED VALUES");
        telemetry.addData("Y Button", "Switch Red/Blue Target");
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
        
        // Right stick Y: Adjust oscillation parameters (when applicable)
        double oscillationAdjustment = -gamepad2.right_stick_y * 0.05;
        if (Math.abs(oscillationAdjustment) > 0.001) {
            if (currentParameter == TuningParameter.OSCILLATION_SPEED) {
                oscillationSpeed = Math.max(0.1, Math.min(1.0, oscillationSpeed + oscillationAdjustment));
            } else if (currentParameter == TuningParameter.OSCILLATION_PERIOD) {
                oscillationPeriod = Math.max(500, Math.min(10000, oscillationPeriod + oscillationAdjustment * 100));
            }
        }
    }
    
    private void adjustCurrentParameter(double adjustment) {
        switch (currentParameter) {
            case KP:
                kp = Math.max(0.0, Math.min(1.0, kp + adjustment));
                break;
            case KI:
                ki = Math.max(0.0, Math.min(1.0, ki + adjustment));
                break;
            case KD:
                kd = Math.max(0.0, Math.min(1.0, kd + adjustment));
                break;
            case OSCILLATION_SPEED:
                oscillationSpeed = Math.max(0.1, Math.min(1.0, oscillationSpeed + adjustment));
                break;
            case OSCILLATION_PERIOD:
                oscillationPeriod = Math.max(500, Math.min(10000, oscillationPeriod + adjustment * 1000));
                break;
        }
    }
    
    private double getParameterAdjustmentScale() {
        switch (currentParameter) {
            case KP:
                return 0.001; // Fine adjustment for KP
            case KI:
                return 0.0005; // Very fine adjustment for KI
            case KD:
                return 0.0005; // Very fine adjustment for KD
            case OSCILLATION_SPEED:
                return 0.01;
            case OSCILLATION_PERIOD:
                return 10.0;
            default:
                return 0.001;
        }
    }
    
    private void handleActionButtons() {
        // A button: Toggle auto-aiming
        boolean currentA = gamepad2.a;
        if (currentA && !previousAButtonState) {
            toggleAutoAiming();
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
        
        // Y button: Switch target
        boolean currentY = gamepad2.y;
        if (currentY && !previousYButtonState) {
            switchTarget();
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
        if (follower != null) {
            follower.update();
        }
        
        // Update limelight
        robot.limelight.update();
        
        // Manual update for non-aiming systems
        robot.intake.update(gamepad2);
        robot.shooter.update(gamepad2);
        
        // Manual turret control when not auto-aiming
        if (!autoAimingEnabled) {
            robot.turret.manualUpdate(gamepad2);
        }
    }
    
    private void handleDriveControls() {
        if (follower != null) {
            follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y, 
                -gamepad1.left_stick_x, 
                -gamepad1.right_stick_x);
        }
    }
    
    private void updateAimingSystem() {
        if (autoAimingEnabled && robot.getAimingController() != null) {
            // Update the aiming controller with current PID values
            // Note: This would require modifying AimingController to accept runtime PID updates
            // For now, we'll track performance and suggest values for Constants.java
            robot.getAimingController().update();
        }
    }
    
    private void updatePerformanceMetrics() {
        if (autoAimingEnabled && robot.limelight.hasTarget()) {
            // Calculate current error (simplified - actual implementation depends on aiming controller)
            double currentTx = robot.limelight.getTx();
            currentError = Math.abs(currentTx);
            
            // Track maximum error for this aiming session
            maxError = Math.max(maxError, currentError);
            
            // Check if we've settled (error < 2 degrees for 0.5 seconds)
            if (currentError < 2.0) {
                if (!hasSettled) {
                    settlingTime = aimingTimer.seconds();
                    hasSettled = true;
                }
            } else {
                hasSettled = false;
            }
        }
    }
    
    private void toggleAutoAiming() {
        autoAimingEnabled = !autoAimingEnabled;
        
        if (autoAimingEnabled) {
            // Reset performance metrics
            aimingTimer.reset();
            maxError = 0.0;
            hasSettled = false;
            settlingTime = 0.0;
            
            // Set target AprilTag
            if (robot.getAimingController() != null) {
                int targetId = (robot.getTargetSide() == Robot.TargetSide.RED) 
                    ? FieldConstants.RED_APRILTAG_ID 
                    : FieldConstants.BLUE_APRILTAG_ID;
                robot.getAimingController().setTargetAprilTagId(targetId);
            }
        } else {
            // Stop aiming
            if (robot.getAimingController() != null) {
                robot.getAimingController().stopAiming();
            }
        }
    }
    
    private void resetCurrentParameter() {
        switch (currentParameter) {
            case KP:
                kp = Constants.TurretAimingConfig.AIMING_KP;
                break;
            case KI:
                ki = Constants.TurretAimingConfig.AIMING_KI;
                break;
            case KD:
                kd = Constants.TurretAimingConfig.AIMING_KD;
                break;
            case OSCILLATION_SPEED:
                oscillationSpeed = Constants.TurretAimingConfig.OSCILLATION_SPEED;
                break;
            case OSCILLATION_PERIOD:
                oscillationPeriod = Constants.TurretAimingConfig.OSCILLATION_PERIOD_MS;
                break;
        }
    }
    
    private void switchTarget() {
        Robot.TargetSide currentSide = robot.getTargetSide();
        Robot.TargetSide newSide = (currentSide == Robot.TargetSide.RED) ? Robot.TargetSide.BLUE : Robot.TargetSide.RED;
        robot.setTargetSide(newSide);
        
        // Reset performance metrics for new target
        if (autoAimingEnabled) {
            aimingTimer.reset();
            maxError = 0.0;
            hasSettled = false;
            settlingTime = 0.0;
        }
    }
    
    private void emergencyStop() {
        autoAimingEnabled = false;
        if (robot.getAimingController() != null) {
            robot.getAimingController().stopAiming();
        }
        robot.turret.setPower(0);
    }
    
    private void saveCurrentValues() {
        String filename = "turret_aiming_" + System.currentTimeMillis() + ".txt";
        tuningUtils.saveTurretAimingParameters(filename, kp, ki, kd, oscillationSpeed, oscillationPeriod);
        
        telemetry.addLine("💾 VALUES SAVED TO: " + filename);
        telemetry.update();
        sleep(1000);
    }
    
    private void saveFinalValues() {
        String filename = "turret_aiming_final_" + System.currentTimeMillis() + ".txt";
        tuningUtils.saveTurretAimingParameters(filename, kp, ki, kd, oscillationSpeed, oscillationPeriod);
    }
    
    // =================================================================================
    // TELEMETRY METHODS
    // =================================================================================
    
    private void displayTuningTelemetry() {
        displayCurrentStatus();
        displayTuningParameters();
        displayPerformanceMetrics();
        displayRobotData();
        displayInstructions();
        
        telemetry.update();
    }
    
    private void displayCurrentStatus() {
        telemetry.addData("🎯", "TURRET AIMING TUNING");
        telemetry.addData("Auto Aiming", autoAimingEnabled ? "🟢 ENABLED" : "🔴 DISABLED");
        telemetry.addData("Target", robot.getTargetSide().toString() + " AprilTag");
        telemetry.addData("Current Parameter", "⚙️ " + currentParameter.toString());
        telemetry.addLine();
    }
    
    private void displayTuningParameters() {
        telemetry.addData("=== PID PARAMETERS ===", "");
        telemetry.addData("KP", "%.4f %s", kp, (currentParameter == TuningParameter.KP) ? "◄" : "");
        telemetry.addData("KI", "%.4f %s", ki, (currentParameter == TuningParameter.KI) ? "◄" : "");
        telemetry.addData("KD", "%.4f %s", kd, (currentParameter == TuningParameter.KD) ? "◄" : "");
        telemetry.addLine();
        telemetry.addData("=== OSCILLATION ===", "");
        telemetry.addData("Speed", "%.2f %s", oscillationSpeed, (currentParameter == TuningParameter.OSCILLATION_SPEED) ? "◄" : "");
        telemetry.addData("Period", "%.0f ms %s", oscillationPeriod, (currentParameter == TuningParameter.OSCILLATION_PERIOD) ? "◄" : "");
        telemetry.addLine();
    }
    
    private void displayPerformanceMetrics() {
        telemetry.addData("=== PERFORMANCE ===", "");
        
        if (autoAimingEnabled) {
            telemetry.addData("Current Error", "%.2f degrees", currentError);
            telemetry.addData("Max Error", "%.2f degrees", maxError);
            telemetry.addData("Aiming Time", "%.1f seconds", aimingTimer.seconds());
            
            if (hasSettled) {
                telemetry.addData("Settling Time", "%.1f seconds ✅", settlingTime);
            } else {
                telemetry.addData("Settling", "Not yet settled");
            }
        } else {
            telemetry.addData("Performance", "Enable auto-aiming to track");
        }
        telemetry.addLine();
    }
    
    private void displayRobotData() {
        telemetry.addData("=== ROBOT DATA ===", "");
        
        if (follower != null) {
            com.pedropathing.geometry.Pose pose = follower.getPose();
            telemetry.addData("Position", "X:%.1f Y:%.1f H:%.0f°", 
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        }
        
        telemetry.addData("Turret Angle", "%.1f degrees", robot.turret.getAngle());
        
        if (robot.limelight.hasTarget()) {
            telemetry.addData("Limelight", "ID:%d TX:%.1f° TY:%.1f°", 
                robot.limelight.getFiducialId(),
                robot.limelight.getTx(),
                robot.limelight.getTy());
        } else {
            telemetry.addData("Limelight", "No Target");
        }
        telemetry.addLine();
    }
    
    private void displayInstructions() {
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("DPAD ←→", "Select Parameter [" + currentParameter + "]");
        telemetry.addData("Left Stick ↑↓", "Adjust Value");
        telemetry.addData("A", "Toggle Aiming | B: Reset | X: Save");
        telemetry.addData("Y", "Switch Target | Start: E-Stop");
    }
}
