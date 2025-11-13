package org.firstinspires.ftc.teamcode.TeleOps.tuning;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.constants.MecanumConstants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.TeleOps.tuning.TuningUtils;

/**
 * Real-time tuning OpMode for distance-based shooting parameters.
 * Allows live adjustment of RPM values and hood positions for different distances.
 * 
 * Controls:
 * Gamepad 1: Drive robot to different distances
 * Gamepad 2: Tuning controls
 * - DPAD UP/DOWN: Select distance breakpoint to tune
 * - Left Stick Y: Adjust RPM for selected distance
 * - Right Stick Y: Adjust hood position for selected distance
 * - A: Test shoot with current parameters
 * - B: Reset current breakpoint to default
 * - X: Save all tuned values to file
 * - Y: Toggle between RPM and Power tuning modes
 */
@TeleOp(name = "🎯 Shooting Tuning", group = "Tuning")
public class ShootingTuningOpMode extends LinearOpMode {
    
    // =================================================================================
    // SUBSYSTEM INSTANCES
    // =================================================================================
    
    private Robot robot;
    private Follower follower;
    private TuningUtils tuningUtils;
    
    // =================================================================================
    // TUNING STATE VARIABLES
    // =================================================================================
    
    // Tunable parameter arrays (copies of constants that we can modify)
    private double[] distanceBreakpoints;
    private double[] rpmValues;
    private double[] hoodValues;
    
    // Current tuning state
    private int currentBreakpointIndex = 4; // Start with middle distance
    private boolean rpmTuningMode = true; // true = RPM tuning, false = Power tuning
    
    // Input tracking
    private boolean previousDpadUpState = false;
    private boolean previousDpadDownState = false;
    private boolean previousAButtonState = false;
    private boolean previousBButtonState = false;
    private boolean previousXButtonState = false;
    private boolean previousYButtonState = false;
    
    // Timing for input sensitivity
    private ElapsedTime inputTimer = new ElapsedTime();
    private static final double INPUT_DEBOUNCE_MS = 150;
    
    // Test shooting state
    private ElapsedTime shootTimer = new ElapsedTime();
    private boolean isTesting = false;
    
    // =================================================================================
    // MAIN OPMODE METHODS
    // =================================================================================
    
    @Override
    public void runOpMode() {
        initializeRobot();
        initializeTuningArrays();
        
        telemetry.addData("🎯", "SHOOTING TUNING MODE");
        telemetry.addData("Status", "Ready to tune distance-based shooting!");
        telemetry.addLine();
        telemetry.addData("📋 CONTROLS 📋", "");
        telemetry.addData("Gamepad 1", "Drive to different distances");
        telemetry.addData("DPAD ↑↓", "Select distance breakpoint");
        telemetry.addData("Left Stick ↑↓", "Adjust RPM/Power");
        telemetry.addData("Right Stick ↑↓", "Adjust Hood Position");
        telemetry.addData("A Button", "Test Shoot");
        telemetry.addData("B Button", "Reset Current Values");
        telemetry.addData("X Button", "💾 SAVE TUNED VALUES");
        telemetry.addData("Y Button", "Toggle RPM/Power Mode");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update robot systems
            updateRobotSystems();
            
            // Handle tuning inputs
            handleTuningInputs();
            
            // Handle drive controls
            handleDriveControls();
            
            // Apply current tuned parameters
            applyTunedParameters();
            
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
        robot.init(hardwareMap, Robot.AimingMode.ENHANCED, Robot.ShootingMode.AUTOMATIC);
        
        tuningUtils = new TuningUtils();
        
        // Enable RPM control for more precise tuning
        robot.shooter.setRPMControlEnabled(rpmTuningMode);
    }
    
    private void initializeTuningArrays() {
        // Create working copies of the constants that we can modify
        distanceBreakpoints = Constants.AutoShootingConfig.DISTANCE_BREAKPOINTS.clone();
        rpmValues = Constants.AutoShootingConfig.SHOOTER_RPM_VALUES.clone();
        hoodValues = Constants.AutoShootingConfig.HOOD_POSITION_VALUES.clone();
        
        // Clamp starting index to valid range
        currentBreakpointIndex = Math.max(0, Math.min(distanceBreakpoints.length - 1, currentBreakpointIndex));
    }
    
    // =================================================================================
    // INPUT HANDLING METHODS
    // =================================================================================
    
    private void handleTuningInputs() {
        // Breakpoint selection (DPAD UP/DOWN)
        handleBreakpointSelection();
        
        // Parameter adjustment (analog sticks)
        handleParameterAdjustment();
        
        // Action buttons
        handleActionButtons();
    }
    
    private void handleBreakpointSelection() {
        boolean currentDpadUp = gamepad2.dpad_up;
        boolean currentDpadDown = gamepad2.dpad_down;
        
        if (currentDpadUp && !previousDpadUpState && inputTimer.milliseconds() > INPUT_DEBOUNCE_MS) {
            currentBreakpointIndex = Math.min(distanceBreakpoints.length - 1, currentBreakpointIndex + 1);
            inputTimer.reset();
        }
        
        if (currentDpadDown && !previousDpadDownState && inputTimer.milliseconds() > INPUT_DEBOUNCE_MS) {
            currentBreakpointIndex = Math.max(0, currentBreakpointIndex - 1);
            inputTimer.reset();
        }
        
        previousDpadUpState = currentDpadUp;
        previousDpadDownState = currentDpadDown;
    }
    
    private void handleParameterAdjustment() {
        // Left stick Y: Adjust RPM/Power (more sensitive for fine-tuning)
        double rpmAdjustment = -gamepad2.left_stick_y * 20.0; // 20 RPM per stick unit
        if (Math.abs(rpmAdjustment) > 0.1) {
            rpmValues[currentBreakpointIndex] = Math.max(
                Constants.AutoShootingConfig.MIN_SHOOTER_RPM,
                Math.min(Constants.AutoShootingConfig.MAX_SHOOTER_RPM,
                        rpmValues[currentBreakpointIndex] + rpmAdjustment));
        }
        
        // Right stick Y: Adjust hood position (very sensitive for precision)
        double hoodAdjustment = -gamepad2.right_stick_y * 0.005; // 0.005 per stick unit
        if (Math.abs(hoodAdjustment) > 0.001) {
            hoodValues[currentBreakpointIndex] = Math.max(
                Constants.AutoShootingConfig.MIN_HOOD_POSITION,
                Math.min(Constants.AutoShootingConfig.MAX_HOOD_POSITION,
                        hoodValues[currentBreakpointIndex] + hoodAdjustment));
        }
    }
    
    private void handleActionButtons() {
        // A button: Test shoot
        boolean currentA = gamepad2.a;
        if (currentA && !previousAButtonState) {
            testShoot();
        }
        previousAButtonState = currentA;
        
        // B button: Reset current breakpoint
        boolean currentB = gamepad2.b;
        if (currentB && !previousBButtonState) {
            resetCurrentBreakpoint();
        }
        previousBButtonState = currentB;
        
        // X button: Save values
        boolean currentX = gamepad2.x;
        if (currentX && !previousXButtonState) {
            saveCurrentValues();
        }
        previousXButtonState = currentX;
        
        // Y button: Toggle tuning mode
        boolean currentY = gamepad2.y;
        if (currentY && !previousYButtonState) {
            toggleTuningMode();
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
        
        // Update robot without launch sequence for manual control
        robot.manualUpdate(gamepad2);
        
        // Update shooting controller if available
        if (robot.getShootingController() != null) {
            robot.getShootingController().update();
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
    
    private void applyTunedParameters() {
        // Set the current distance and let the system calculate parameters
        double currentDistance = distanceBreakpoints[currentBreakpointIndex];
        robot.shooter.setTargetDistance(currentDistance);
        
        // Override with our tuned values
        if (rpmTuningMode) {
            robot.shooter.setRPM(rpmValues[currentBreakpointIndex]);
        } else {
            robot.shooter.setPower(rpmValues[currentBreakpointIndex] / 5000.0); // Convert RPM to approximate power
        }
        robot.shooter.setHoodPosition(hoodValues[currentBreakpointIndex]);
    }
    
    private void testShoot() {
        isTesting = true;
        shootTimer.reset();
        
        // Apply current parameters and shoot for a brief period
        if (rpmTuningMode) {
            robot.shooter.setRPM(rpmValues[currentBreakpointIndex]);
        } else {
            robot.shooter.setPower(rpmValues[currentBreakpointIndex] / 5000.0);
        }
    }
    
    private void resetCurrentBreakpoint() {
        // Reset to original constant values
        rpmValues[currentBreakpointIndex] = Constants.AutoShootingConfig.SHOOTER_RPM_VALUES[currentBreakpointIndex];
        hoodValues[currentBreakpointIndex] = Constants.AutoShootingConfig.HOOD_POSITION_VALUES[currentBreakpointIndex];
    }
    
    private void saveCurrentValues() {
        String filename = "shooting_tuning_" + System.currentTimeMillis() + ".txt";
        tuningUtils.saveShootingParameters(filename, distanceBreakpoints, rpmValues, hoodValues, rpmTuningMode);
        
        // Also update telemetry with save confirmation
        telemetry.addLine("💾 VALUES SAVED TO: " + filename);
        telemetry.update();
        sleep(1000); // Brief pause to show save message
    }
    
    private void toggleTuningMode() {
        rpmTuningMode = !rpmTuningMode;
        robot.shooter.setRPMControlEnabled(rpmTuningMode);
    }
    
    private void saveFinalValues() {
        // Auto-save when OpMode ends
        String filename = "shooting_final_" + System.currentTimeMillis() + ".txt";
        tuningUtils.saveShootingParameters(filename, distanceBreakpoints, rpmValues, hoodValues, rpmTuningMode);
    }
    
    // =================================================================================
    // TELEMETRY METHODS
    // =================================================================================
    
    private void displayTuningTelemetry() {
        displayCurrentStatus();
        displayTuningParameters();
        displayRobotData();
        displayInstructions();
        
        // Stop test shooting after timeout
        if (isTesting && shootTimer.seconds() > 2.0) {
            robot.shooter.setPower(0);
            robot.shooter.setRPM(0);
            isTesting = false;
        }
        
        telemetry.update();
    }
    
    private void displayCurrentStatus() {
        telemetry.addData("🎯", "SHOOTING TUNING MODE");
        telemetry.addData("Tuning Mode", rpmTuningMode ? "RPM Control" : "Power Control");
        telemetry.addData("Test Shooting", isTesting ? "🔥 FIRING" : "Ready");
        telemetry.addLine();
    }
    
    private void displayTuningParameters() {
        telemetry.addData("=== CURRENT BREAKPOINT ===", "");
        telemetry.addData("Distance", "%.1f inches [%d/%d]", 
            distanceBreakpoints[currentBreakpointIndex], 
            currentBreakpointIndex + 1, 
            distanceBreakpoints.length);
        
        if (rpmTuningMode) {
            telemetry.addData("Current RPM", "%.0f (Target: %.0f)", 
                robot.shooter.getCurrentRPM(), 
                rpmValues[currentBreakpointIndex]);
            telemetry.addData("RPM Stable", robot.shooter.isRPMStable(rpmValues[currentBreakpointIndex]) ? "✅" : "❌");
        } else {
            telemetry.addData("Current Power", "%.2f", robot.shooter.getMotorPower());
        }
        
        telemetry.addData("Hood Position", "%.3f", hoodValues[currentBreakpointIndex]);
        telemetry.addData("Current Hood", "%.3f", robot.shooter.getServoPosition());
        telemetry.addLine();
    }
    
    private void displayRobotData() {
        telemetry.addData("=== ROBOT DATA ===", "");
        
        if (follower != null) {
            com.pedropathing.geometry.Pose pose = follower.getPose();
            telemetry.addData("Robot Position", "X:%.1f Y:%.1f H:%.0f°", 
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        }
        
        telemetry.addData("Limelight Target", robot.limelight.hasTarget() ? 
            String.format("ID:%d Dist:%.1f\"", robot.limelight.getFiducialId(), robot.limelight.getDistanceToTarget()) : 
            "None");
        telemetry.addLine();
    }
    
    private void displayInstructions() {
        telemetry.addData("=== QUICK REFERENCE ===", "");
        telemetry.addData("DPAD ↑↓", "Select Distance [" + (currentBreakpointIndex + 1) + "/" + distanceBreakpoints.length + "]");
        telemetry.addData("Left Stick ↑↓", rpmTuningMode ? "Adjust RPM" : "Adjust Power");
        telemetry.addData("Right Stick ↑↓", "Adjust Hood");
        telemetry.addData("A", "Test Shoot | B: Reset | X: Save | Y: Toggle Mode");
    }
}
