package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.Constants;

public class Limelight {
    // Constants for the camera's physical position
    private final double cameraHeight;
    private final double cameraAngle;

    public Limelight(HardwareMap hardwareMap, String limelightName) {
        // Initialization logic for the Limelight
        // e.g., webcam = hardwareMap.get(WebcamName.class, limelightName);

        // Store the physical constants from the config file
        this.cameraHeight = Constants.LimelightConfig.LIMELIGHT_HEIGHT;
        this.cameraAngle = Constants.LimelightConfig.LIMELIGHT_ANGLE;
    }

    // You can now use cameraHeight and cameraAngle in your distance calculations

    public double getTargetX() {
        // Placeholder
        return 0.0;
    }

    public double getTargetY() {
        // Placeholder
        return 0.0;
    }

    public boolean hasTarget() {
        // Placeholder
        return false;
    }

    public double getTargetArea() {
        // Placeholder
        return 0.0;
    }
}
