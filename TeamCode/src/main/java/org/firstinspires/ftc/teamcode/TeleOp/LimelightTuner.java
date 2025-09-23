package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.LimelightConfig;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.utils.PatternIdentifier; // Added for pattern identification

import java.util.Locale;

@TeleOp(name = "Limelight Tuner (Panels)", group = "Vision")
public class LimelightTuner extends OpMode {
    private TelemetryManager telemetryM;
    private Limelight limelight;
    // private LimelightConfig limelightConfig; // This can be removed if always using default

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        // Initialize Limelight with a specific config, e.g., the default one
        limelight = new Limelight(hardwareMap, LimelightConfig.LIMELIGHT_DEFAULT);
        limelight.start(); // Start Limelight polling
    }

    @Override
    public void loop() {
        limelight.updateResult(); // Fetch the latest data from Limelight

        telemetryM.debug("Has Targets: " + limelight.hasTargets());
        int tagId = limelight.getBestTagId();
        telemetryM.debug("Best Tag ID: " + tagId);
        
        // Get and display the pattern description based on the Tag ID
        String patternDescription = PatternIdentifier.getPatternDescription(tagId);
        telemetryM.debug("Detected Pattern: " + patternDescription);

        telemetryM.debug("Best Tag Name: " + limelight.getBestTagName());
        telemetryM.debug("tx: " + String.format(Locale.US, "%.2f", limelight.getTx()));
        telemetryM.debug("ty: " + String.format(Locale.US, "%.2f", limelight.getTy()));
        
        Pose3D pose = limelight.getBotPose();
        if (pose != null) {
            telemetryM.debug(String.format(Locale.US, "BotPose X: %.2f m", pose.getPosition().x));
            telemetryM.debug(String.format(Locale.US, "BotPose Y: %.2f m", pose.getPosition().y));
            telemetryM.debug(String.format(Locale.US, "BotPose Z: %.2f m", pose.getPosition().z));
            // You can also display rotation if needed:
            // telemetryM.debug(String.format(Locale.US, "BotPose Roll: %.2f deg", Math.toDegrees(pose.roll)));
            // telemetryM.debug(String.format(Locale.US, "BotPose Pitch: %.2f deg", Math.toDegrees(pose.pitch)));
            // telemetryM.debug(String.format(Locale.US, "BotPose Yaw: %.2f deg", Math.toDegrees(pose.yaw)));
        } else {
            telemetryM.debug("BotPose X: N/A");
            telemetryM.debug("BotPose Y: N/A");
            telemetryM.debug("BotPose Z: N/A");
        }
        telemetryM.update(telemetry);
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop(); // Stop Limelight polling
        }
    }
}
