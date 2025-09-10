package org.firstinspires.ftc.teamcode.config;

public enum LimelightConfig {
    LIMELIGHT_DEFAULT("limelight");

    private final String hardwareName;

    LimelightConfig(String hardwareName) {
        this.hardwareName = hardwareName;
    }

    public String getHardwareName() {
        return hardwareName;
    }
}
