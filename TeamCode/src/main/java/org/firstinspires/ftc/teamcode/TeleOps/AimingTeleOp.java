package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.AimingController;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

/**
 * AimingTeleOp tests the automatic turret aiming system.
 * 
 * Gamepad 1: Standard drivetrain control
 * Gamepad 2: Turret control and aiming mode toggle
 * 
 * Controls:
 * - Gamepad2 A: Toggle between AIMING and MANUAL mode
 * - Gamepad2 DPAD_UP: Target Red AprilTag (ID 20)
 * - Gamepad2 DPAD_DOWN: Target Blue AprilTag (ID 24)
 * - Gamepad2 Left/Right Bumpers: Manual turret control (when in MANUAL mode)
 */
@TeleOp(name = "Aiming TeleOp", group = "Test")
public class AimingTeleOp extends LinearOpMode {
    
    private Follower follower;
    private Turret turret;
    private Limelight limelight;
    private AimingController aimingController;
    
    private boolean aimingMode = false;
    private boolean previousAButtonState = false;
    private boolean previousDpadUpState = false;
    private boolean previousDpadDownState = false;
    
    @Override
    public void runOpMode() {
        // Initialize hardware and subsystems
        try {
            follower = Constants.createFollower(hardwareMap);
            turret = new Turret(hardwareMap);
            limelight = new Limelight(hardwareMap);
            aimingController = new AimingController(turret, limelight, follower);
            
            // Set initial pose (adjust as needed for your field setup)
            follower.setStartingPose(new Pose(0, 0, 0));
            
            telemetry.addData("Status", "Initialized successfully");
            telemetry.addData("Note", "Press START to begin");
            telemetry.update();
            
        } catch (Exception e) {
            telemetry.addData("Initialization Error", e.getMessage());
            telemetry.update();
            return;
        }
        
        waitForStart();
        
        if (opModeIsActive()) {
            telemetry.addData("Status", "Running - Press STOP to end");
            telemetry.update();
        }
        
        while (opModeIsActive()) {
            try {
                // Update odometry
                follower.update();
                
                // --- Gamepad 1: Standard drivetrain control ---
                double drive = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x; 
                double turn = gamepad1.right_stick_x;
                
                // Apply drive commands
                follower.setTeleOpDrive(drive, strafe, turn, true);
                
                // --- Gamepad 2: Turret control and mode switching ---
                
                // Toggle aiming mode with A button
                boolean currentAButtonState = gamepad2.a;
                if (currentAButtonState && !previousAButtonState) {
                    aimingMode = !aimingMode;
                    if (aimingMode) {
                        turret.setState(Turret.TurretState.AIMING);
                        telemetry.addData("Mode Switch", "AIMING mode activated");
                    } else {
                        turret.setState(Turret.TurretState.MANUAL);
                        aimingController.stopAiming();
                        telemetry.addData("Mode Switch", "MANUAL mode activated");
                    }
                }
                previousAButtonState = currentAButtonState;
                
                // AprilTag selection (when in aiming mode)
                if (aimingMode) {
                    // DPAD controls for AprilTag selection
                    if (gamepad2.dpad_up && !previousDpadUpState) {
                        aimingController.setTargetAprilTagId(20);
                        telemetry.addData("Target", "Red AprilTag (ID 20) selected");
                    }
                    if (gamepad2.dpad_down && !previousDpadDownState) {
                        aimingController.setTargetAprilTagId(24);
                        telemetry.addData("Target", "Blue AprilTag (ID 24) selected");
                    }
                    
                    // Update aiming controller
                    aimingController.update();
                } else {
                    // Manual turret control
                    turret.manualUpdate(gamepad2);
                }
                
                // Store previous DPAD states
                previousDpadUpState = gamepad2.dpad_up;
                previousDpadDownState = gamepad2.dpad_down;
                
                // --- Telemetry ---
                Pose currentPose = follower.getPose();
                
                telemetry.addData("=== ROBOT STATUS ===", "");
                telemetry.addData("Mode", aimingMode ? "AIMING" : "MANUAL");
                telemetry.addData("Robot X", "%.2f inches", currentPose.getX());
                telemetry.addData("Robot Y", "%.2f inches", currentPose.getY());
                telemetry.addData("Robot Heading", "%.1f degrees", Math.toDegrees(currentPose.getHeading()));
                
                telemetry.addData("=== TURRET STATUS ===", "");
                telemetry.addData("Turret State", turret.getState());
                telemetry.addData("Turret Angle", "%.1f degrees", turret.getAngle());
                if (aimingMode) {
                    telemetry.addData("Target AprilTag", aimingController.getTargetAprilTagId());
                    telemetry.addData("Target Field Angle", "%.1f degrees", 
                                    Math.toDegrees(turret.getTargetFieldAngle()));
                }
                
                telemetry.addData("=== VISION STATUS ===", "");
                telemetry.addData("Limelight Target", limelight.hasTarget() ? "YES" : "NO");
                if (limelight.hasTarget()) {
                    telemetry.addData("AprilTag ID", limelight.getFiducialId());
                    telemetry.addData("TX", "%.2f degrees", limelight.getTx());
                    telemetry.addData("TY", "%.2f degrees", limelight.getTy());
                    telemetry.addData("Distance", "%.1f inches", limelight.getDistanceToTarget());
                }
                
                telemetry.addData("=== CONTROLS ===", "");
                telemetry.addData("GP2 A", "Toggle AIMING/MANUAL mode");
                telemetry.addData("GP2 DPAD UP", "Target Red AprilTag (20)");
                telemetry.addData("GP2 DPAD DOWN", "Target Blue AprilTag (24)");
                telemetry.addData("GP2 Bumpers", "Manual turret (MANUAL mode)");
                
                telemetry.update();
                
            } catch (Exception e) {
                telemetry.addData("Runtime Error", e.getMessage());
                telemetry.update();
                // Continue running to allow for recovery
            }
        }
        
        // Cleanup
        try {
            if (limelight != null) {
                limelight.stop();
            }
        } catch (Exception e) {
            // Ignore cleanup errors
        }
    }
}
