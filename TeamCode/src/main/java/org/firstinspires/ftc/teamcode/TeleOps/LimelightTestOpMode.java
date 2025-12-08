package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.Constants;

/**
 * Comprehensive test OpMode for Limelight 3A
 * Tests all the functionality used by AutoAimingTurret class
 * 
 * Controls:
 * - Gamepad1 A: Toggle pipeline (if multiple pipelines configured)
 * - Displays continuous telemetry of all Limelight data
 */
@TeleOp(name = "Limelight 3A Test", group = "Testing")
public class LimelightTestOpMode extends LinearOpMode {

    private Limelight3A limelight;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime dataTimer = new ElapsedTime();
    
    // Statistics tracking
    private int validResults = 0;
    private int totalResults = 0;
    private int botPoseValidCount = 0;
    
    @Override
    public void runOpMode() throws InterruptedException {
        
        // Initialize hardware
        try {
            limelight = hardwareMap.get(Limelight3A.class, Constants.HardwareConfig.LIMELIGHT_NAME);
            telemetry.addData("Limelight", "Successfully initialized");
        } catch (Exception e) {
            telemetry.addData("Limelight Error", e.getMessage());
            telemetry.addData("Check", "Hardware map name: '" + Constants.HardwareConfig.LIMELIGHT_NAME + "'");
            telemetry.update();
            
            // Wait for start anyway to show error
            waitForStart();
            return;
        }
        
        telemetry.addData("Status", "Limelight 3A Test Ready");
        telemetry.addData("Purpose", "This will test AutoAimingTurret compatibility");
        telemetry.addData("Instructions", "Press PLAY to start testing...");
        telemetry.update();
        
        waitForStart();
        runtime.reset();
        dataTimer.reset();
        
        while (opModeIsActive()) {
            
            // Get latest result from Limelight
            LLResult result = limelight.getLatestResult();
            totalResults++;
            
            // Clear previous telemetry
            telemetry.clear();
            
            // === HEADER ===
            telemetry.addData("LIMELIGHT 3A TEST", "Runtime: %.1fs", runtime.seconds());
            telemetry.addLine("=========================================");
            
            // === CONNECTION STATUS ===
            if (result != null) {
                validResults++;
                telemetry.addData("Connection", "CONNECTED");
                telemetry.addData("Data Rate", "%.1f Hz", totalResults / runtime.seconds());
                telemetry.addData("Success Rate", "%.1f%% (%d/%d)", 
                    (double)validResults / totalResults * 100, validResults, totalResults);
            } else {
                telemetry.addData("Connection", "NO DATA");
                telemetry.addData("Check", "Limelight power/network");
            }
            
            telemetry.addLine();
            
            // === RESULT VALIDITY ===
            if (result != null && result.isValid()) {
                telemetry.addData("Result Status", "VALID");
                
                // === APRILTAG DETECTION ===
                if (result.getFiducialResults() != null && result.getFiducialResults().length > 0) {
                    LLResultTypes.FiducialResult[] tags = result.getFiducialResults();
                    telemetry.addData("AprilTags Found", "%d tags", tags.length);
                    
                    // Show details for first few tags
                    for (int i = 0; i < Math.min(3, tags.length); i++) {
                        LLResultTypes.FiducialResult tag = tags[i];
                        telemetry.addData("  Tag " + tag.getFiducialId(), 
                            "X:%.2f Y:%.2f Area:%.1f", 
                            tag.getTargetXDegrees(), 
                            tag.getTargetYDegrees(),
                            tag.getTargetArea());
                    }
                } else {
                    telemetry.addData("AprilTags Found", "NONE DETECTED");
                    telemetry.addData("Make sure", "AprilTags are in view");
                }
                
                telemetry.addLine();
                
                // === BOTPOSE DATA (Used by AutoAimingTurret) ===
                Pose3D botPose = result.getBotpose();
                if (botPose != null) {
                    botPoseValidCount++;
                    telemetry.addData("BotPose Status", "AVAILABLE");
                    telemetry.addData("Robot Position", "X:%.3f Y:%.3f", 
                        botPose.getPosition().x, 
                        botPose.getPosition().y);
                    telemetry.addData("Robot Rotation", "Yaw:%.2f degrees", 
                        Math.toDegrees(botPose.getOrientation().getYaw()));
                    telemetry.addData("Full Orientation", "R:%.1f P:%.1f Y:%.1f", 
                        Math.toDegrees(botPose.getOrientation().getRoll()),
                        Math.toDegrees(botPose.getOrientation().getPitch()),
                        Math.toDegrees(botPose.getOrientation().getYaw()));
                    
                    // === COORDINATE VALIDATION ===
                    double x = botPose.getPosition().x;
                    double y = botPose.getPosition().y;
                    if (Math.abs(x) > 200 || Math.abs(y) > 200) {
                        telemetry.addData("Position Warning", "Coordinates seem large - check field setup");
                    }
                } else {
                    telemetry.addData("BotPose Status", "NULL");
                    telemetry.addData("Possible Issues", "No AprilTags or bad field setup");
                }
                
                telemetry.addLine();
                
                // === RAW LIMELIGHT DATA ===
                telemetry.addData("Raw Data", "");
                telemetry.addData("  Timestamp", result.getTimestampSeconds());
                telemetry.addData("  Latency", "%.1f ms", result.getStaleness());
                
            } else if (result != null) {
                telemetry.addData("Result Status", "INVALID");
                telemetry.addData("Check", "Pipeline settings or lighting");
            } else {
                telemetry.addData("Result Status", "NULL RESULT");
            }
            
            telemetry.addLine();
            
            // === STATISTICS ===
            telemetry.addData("STATISTICS", "");
            telemetry.addData("  Total Samples", totalResults);
            telemetry.addData("  Valid Results", "%.1f%% (%d)", 
                totalResults > 0 ? (double)validResults / totalResults * 100 : 0, validResults);
            telemetry.addData("  BotPose Success", "%.1f%% (%d)", 
                validResults > 0 ? (double)botPoseValidCount / validResults * 100 : 0, botPoseValidCount);
            
            telemetry.addLine();
            
            // === AUTOAIMINGTURRENT COMPATIBILITY ===
            boolean turretReady = (result != null && result.isValid() && result.getBotpose() != null);
            telemetry.addData("AutoAimingTurret", turretReady ? "READY" : "NOT READY");
            if (!turretReady) {
                if (result == null) {
                    telemetry.addData("  Fix", "Check Limelight connection");
                } else if (!result.isValid()) {
                    telemetry.addData("  Fix", "Check pipeline/AprilTag setup");
                } else {
                    telemetry.addData("  Fix", "Need visible AprilTags for BotPose");
                }
            }
            
            telemetry.addLine();
            
            // === CONTROLS ===
            telemetry.addData("CONTROLS", "");
            telemetry.addData("  A Button", "Toggle pipeline (if available)");
            telemetry.addData("  STOP", "End test");
            
            // Handle controls
            if (gamepad1.a && dataTimer.seconds() > 0.5) {
                try {
                    // Try to cycle through pipelines if multiple exist
                    limelight.pipelineSwitch(0); // You can change this to cycle through 0,1,2,etc
                    dataTimer.reset();
                } catch (Exception e) {
                    // Ignore pipeline switching errors
                }
            }
            
            telemetry.update();
            
            // Small delay to prevent overwhelming the system
            sleep(50);
        }
    }
}
