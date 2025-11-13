package org.firstinspires.ftc.teamcode.TeleOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.constants.MecanumConstants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * TeleOp for testing the automatic distance-based shooting system.
 * Features gamepad controls for toggling between manual and automatic shooting modes,
 * and comprehensive telemetry for tuning and debugging.
 */
@TeleOp(name = "Auto Shooting Test", group = "Testing")
public class AutoShootingTeleOp extends LinearOpMode {
    
    // =================================================================================
    // SUBSYSTEM INSTANCES
    // =================================================================================
    
    private Robot robot;
    private Follower follower;
    
    // =================================================================================
    // STATE VARIABLES
    // =================================================================================
    
    // Button press tracking for toggle controls
    private boolean previousYButtonState = false;      // Auto shooting toggle
    private boolean previousXButtonState = false;      // Force parameter update
    private boolean previousBButtonState = false;      // Reset distance history
    private boolean previousAButtonState = false;      // Manual shooting test
    private boolean previousStartButtonState = false;  // RPM/Power control toggle
    
    // =================================================================================
    // MAIN OPMODE METHODS
    // =================================================================================
    
    @Override
    public void runOpMode() {
        // Initialize robot with automatic shooting enabled
        initializeRobot();
        
        telemetry.addData("Status", "Initialized - Auto Shooting Test Ready!");
        telemetry.addData("Controls", "Gamepad 1: Drive | Gamepad 2: Shooting");
        telemetry.addData("Y Button", "Toggle Auto/Manual Shooting");
        telemetry.addData("X Button", "Force Parameter Update");
        telemetry.addData("B Button", "Reset Distance History");
        telemetry.addData("A Button", "Manual Shoot Test");
        telemetry.addData("Start Button", "Toggle RPM/Power Control");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Update follower odometry for accurate position tracking
            if (follower != null) {
                follower.update();
            }
            
            // Handle gamepad inputs for shooting system
            handleShootingControls();
            
            // Drive control with gamepad 1
            handleDriveControls();
            
            // Manual subsystem control with gamepad 2 (when not in auto mode)
            handleManualSubsystemControls();
            
            // Update robot systems
            robot.manualUpdate(gamepad2);
            
            // Display comprehensive telemetry
            displayTelemetry();
            
            telemetry.update();
        }
    }
    
    // =================================================================================
    // INITIALIZATION METHODS
    // =================================================================================
    
    /**
     * Initializes the robot with pedropathing and automatic shooting enabled.
     */
    private void initializeRobot() {
        // Initialize the pedropathing follower
        follower = new Follower(hardwareMap, MecanumConstants.class);
        follower.setStartingPose(new com.pedropathing.geometry.Pose(0, 0, 0));
        
        // Initialize robot with enhanced aiming and automatic shooting
        robot = new Robot(follower);
        robot.init(hardwareMap, Robot.AimingMode.ENHANCED, Robot.ShootingMode.AUTOMATIC);
    }
    
    // =================================================================================
    // CONTROL HANDLING METHODS
    // =================================================================================
    
    /**
     * Handles shooting system controls on gamepad 2.
     */
    private void handleShootingControls() {
        // Toggle between manual and automatic shooting modes
        boolean currentYButtonState = gamepad2.y;
        if (currentYButtonState && !previousYButtonState) {
            if (robot.getShootingMode() == Robot.ShootingMode.MANUAL) {
                robot.setShootingMode(Robot.ShootingMode.AUTOMATIC);
                telemetry.addLine("🎯 AUTOMATIC SHOOTING MODE ACTIVATED");
            } else {
                robot.setShootingMode(Robot.ShootingMode.MANUAL);
                telemetry.addLine("👐 MANUAL SHOOTING MODE ACTIVATED");
            }
        }
        previousYButtonState = currentYButtonState;
        
        // Force parameter update (useful for testing)
        boolean currentXButtonState = gamepad2.x;
        if (currentXButtonState && !previousXButtonState) {
            if (robot.getShootingController() != null) {
                robot.getShootingController().forceParameterUpdate();
                telemetry.addLine("🔄 FORCED PARAMETER UPDATE");
            }
        }
        previousXButtonState = currentXButtonState;
        
        // Reset distance measurement history
        boolean currentBButtonState = gamepad2.b;
        if (currentBButtonState && !previousBButtonState) {
            if (robot.getShootingController() != null) {
                robot.getShootingController().resetDistanceHistory();
                telemetry.addLine("🔄 DISTANCE HISTORY RESET");
            }
        }
        previousBButtonState = currentBButtonState;
        
        // Manual shooting test (works in both modes)
        boolean currentAButtonState = gamepad2.a;
        if (currentAButtonState && !previousAButtonState) {
            // Briefly activate the shooter for testing
            if (robot.shooter.isRPMControlEnabled()) {
                robot.shooter.setRPM(3000); // Test RPM
                telemetry.addLine("🔥 MANUAL RPM TEST: 3000 RPM");
            } else {
                robot.shooter.setPower(0.8);
                telemetry.addLine("🔥 MANUAL POWER TEST: 80%");
            }
        } else if (!currentAButtonState && previousAButtonState) {
            // Stop shooter when button is released
            if (robot.shooter.isRPMControlEnabled()) {
                robot.shooter.setRPM(0);
            } else {
                robot.shooter.setPower(0.0);
            }
        }
        previousAButtonState = currentAButtonState;
        
        // Toggle RPM/Power control mode
        boolean currentStartButtonState = gamepad2.start;
        if (currentStartButtonState && !previousStartButtonState) {
            boolean newMode = !robot.shooter.isRPMControlEnabled();
            robot.shooter.setRPMControlEnabled(newMode);
            if (newMode) {
                telemetry.addLine("⚙️ RPM CONTROL MODE ACTIVATED");
            } else {
                telemetry.addLine("⚙️ POWER CONTROL MODE ACTIVATED");
            }
        }
        previousStartButtonState = currentStartButtonState;
    }
    
    /**
     * Handles drive controls on gamepad 1.
     */
    private void handleDriveControls() {
        if (follower != null) {
            // Use pedropathing's TeleOp drive method for smooth control
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            follower.update();
        }
    }
    
    /**
     * Handles manual subsystem controls when not in automatic shooting mode.
     */
    private void handleManualSubsystemControls() {
        // Only allow manual shooter control if not in automatic mode
        if (robot.getShootingMode() == Robot.ShootingMode.MANUAL) {
            // Intake controls (left trigger/bumper)
            robot.intake.update(gamepad2);
            
            // Turret controls (left/right bumpers)
            robot.turret.manualUpdate(gamepad2);
        } else {
            // Still allow intake and turret control in auto shooting mode
            robot.intake.update(gamepad2);
            robot.turret.manualUpdate(gamepad2);
        }
    }
    
    // =================================================================================
    // TELEMETRY METHODS
    // =================================================================================
    
    /**
     * Displays comprehensive telemetry for debugging and tuning.
     */
    private void displayTelemetry() {
        displayShootingSystemStatus();
        displayRobotPose();
        displayLimelightData();
        displayShooterParameters();
        displaySubsystemStatus();
    }
    
    /**
     * Displays shooting system status and mode information.
     */
    private void displayShootingSystemStatus() {
        telemetry.addData("=== SHOOTING SYSTEM ===", "");
        telemetry.addData("Shooting Mode", robot.getShootingMode().toString());
        
        if (robot.getShootingController() != null) {
            telemetry.addData("Auto Shooting Status", robot.getShootingController().getStatusString());
            telemetry.addData("Ready to Shoot", robot.getShootingController().isReadyToShoot() ? "✅ YES" : "❌ NO");
            telemetry.addData("Has Target", robot.getShootingController().hasTarget() ? "✅ YES" : "❌ NO");
            telemetry.addData("Valid Distance", robot.getShootingController().hasValidDistance() ? "✅ YES" : "❌ NO");
        } else {
            telemetry.addData("Auto Shooting Status", "NOT AVAILABLE (Manual Mode)");
        }
    }
    
    /**
     * Displays robot position information.
     */
    private void displayRobotPose() {
        telemetry.addData("=== ROBOT POSITION ===", "");
        if (follower != null) {
            com.pedropathing.geometry.Pose currentPose = follower.getPose();
            telemetry.addData("Robot X", "%.2f inches", currentPose.getX());
            telemetry.addData("Robot Y", "%.2f inches", currentPose.getY());
            telemetry.addData("Robot Heading", "%.1f degrees", Math.toDegrees(currentPose.getHeading()));
        } else {
            telemetry.addData("Robot Position", "Pedropathing not available");
        }
    }
    
    /**
     * Displays Limelight vision data.
     */
    private void displayLimelightData() {
        telemetry.addData("=== LIMELIGHT DATA ===", "");
        telemetry.addData("Has Target", robot.limelight.hasTarget() ? "✅ YES" : "❌ NO");
        
        if (robot.limelight.hasTarget()) {
            telemetry.addData("Target ID", robot.limelight.getFiducialId());
            telemetry.addData("TX (degrees)", "%.2f", robot.limelight.getTx());
            telemetry.addData("TY (degrees)", "%.2f", robot.limelight.getTy());
            telemetry.addData("Distance", "%.1f inches", robot.limelight.getDistanceToTarget());
            
            if (robot.getShootingController() != null) {
                telemetry.addData("Last Valid Distance", "%.1f inches", robot.getShootingController().getLastValidDistance());
            }
        } else {
            telemetry.addData("Target Info", "No target visible");
        }
    }
    
    /**
     * Displays calculated shooting parameters with RPM support.
     */
    private void displayShooterParameters() {
        telemetry.addData("=== SHOOTER PARAMETERS ===", "");
        telemetry.addData("Shooter State", robot.shooter.getState().toString());
        telemetry.addData("Control Mode", robot.shooter.isRPMControlEnabled() ? "RPM Control" : "Power Control");
        
        // Current values
        if (robot.shooter.isRPMControlEnabled()) {
            double currentRPM = robot.shooter.getCurrentRPM();
            telemetry.addData("Current RPM", currentRPM >= 0 ? String.format("%.0f", currentRPM) : "Not Available");
        } else {
            telemetry.addData("Current Power", "%.2f", robot.shooter.getMotorPower());
        }
        telemetry.addData("Current Hood Pos", "%.3f", robot.shooter.getServoPosition());
        
        // Calculated parameters
        if (robot.shooter.isAutoShootingReady()) {
            telemetry.addData("Target Distance", "%.1f inches", robot.shooter.getTargetDistance());
            
            if (robot.shooter.isRPMControlEnabled()) {
                telemetry.addData("Target RPM", "%.0f", robot.shooter.getCalculatedRPM());
                telemetry.addData("RPM Stable", robot.shooter.isRPMStable(robot.shooter.getCalculatedRPM()) ? "✅ YES" : "❌ NO");
            } else {
                telemetry.addData("Calculated Power", "%.2f", robot.shooter.getCalculatedPower());
            }
            
            telemetry.addData("Calculated Hood", "%.3f", robot.shooter.getCalculatedHoodPosition());
            telemetry.addData("Auto Parameters", robot.shooter.getShootingParametersString());
            telemetry.addData("Detailed Status", robot.shooter.getDetailedStatusString());
        } else {
            telemetry.addData("Auto Parameters", "Not ready / Manual mode");
        }
    }
    
    /**
     * Displays other subsystem status information.
     */
    private void displaySubsystemStatus() {
        telemetry.addData("=== OTHER SUBSYSTEMS ===", "");
        telemetry.addData("Turret Angle", "%.1f degrees", robot.turret.getAngle());
        telemetry.addData("Intake Running", robot.intake.getMotorPower() != 0 ? "✅ YES" : "❌ NO");
        telemetry.addData("Object Detected", robot.intake.isObjectDetected() ? "✅ YES" : "❌ NO");
        telemetry.addData("Launch Sequence", robot.getLaunchSequenceState());
        
        // Performance monitoring
        telemetry.addData("=== PERFORMANCE ===", "");
        telemetry.addData("Loop Time", "%.1f ms", getRuntime() * 1000);
    }
}
