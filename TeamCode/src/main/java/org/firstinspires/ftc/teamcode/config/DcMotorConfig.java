package org.firstinspires.ftc.teamcode.config;

/**
 * Enum mapping for all DC motors used on the robot.
 * Replace the sample entries with your actual configured names.
 * Centralizes hardwareMap string names to avoid scattering literals.
 */
public enum DcMotorConfig {
    // Drivetrain examples (edit to match RC configuration)
    LEFT_FRONT("left_front"),
    LEFT_REAR("left_rear"),
    RIGHT_FRONT("right_front"),
    RIGHT_REAR("right_rear"),

    // Add additional motors here, e.g. intake, lift, arm
    // INTAKE("intake"),
    // LIFT("lift"),
    ;

    private final String hardwareName;

    DcMotorConfig(String hardwareName) {
        this.hardwareName = hardwareName;
    }

    public String getHardwareName() {
        return hardwareName;
    }

    public static DcMotorConfig fromHardwareName(String name) {
        for (DcMotorConfig m : values()) {
            if (m.hardwareName.equals(name)) return m;
        }
        return null;
    }
}


