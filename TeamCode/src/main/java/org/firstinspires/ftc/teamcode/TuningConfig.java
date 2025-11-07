package org.firstinspires.ftc.teamcode;

/**
 * Centralized configuration for tuning values used throughout the robot code.
 *
 * Group sections by subsystem. Update values here to propagate across the project.
 */
public final class TuningConfig {
    private TuningConfig() {}

    // ===============================
    // Drivetrain
    // ===============================
    public static final double TELEOP_NORMAL_SCALE = 1.0;
    public static final double TELEOP_SLOW_SCALE = 0.4;

    // ===============================
    // Controls (deadbands, etc.)
    // ===============================
    public static final double ANALOG_DEADBAND = 0.05;

    // ===============================
    // Odometry (Pinpoint)
    // ===============================
    // Offsets are in millimeters: x = left(+)/right(-) of center, y = forward(+)/back(-) of center
    public static final double PINPOINT_X_OFFSET_MM = -84.0; // EDIT
    public static final double PINPOINT_Y_OFFSET_MM = -168.0; // EDIT
    public static final boolean PINPOINT_FORWARD_ENCODER_FORWARD = true; // EDIT
    public static final boolean PINPOINT_STRAFE_ENCODER_FORWARD = true; // EDIT
    public static final boolean PINPOINT_USE_FOUR_BAR_POD = true; // EDIT

    // ===============================
    // Autonomous examples
    // ===============================
    public static final double AUTO_FORWARD_INCHES = 24.0;
    public static final double AUTO_SQUARE_SIDE_INCHES = 24.0;
    public static final double AUTO_DRIVE_POWER = 0.6;
    public static final double AUTO_DISTANCE_TOLERANCE_IN = 1.0;

    // ===============================
    // Limelight 3A
    // ===============================
    public static final String LIMELIGHT_HOST = "10.0.0.11"; // EDIT: set LL IP or hostname
    public static final java.util.Map<Integer, String> APRILTAG_ID_TO_NAME = new java.util.HashMap<Integer, String>() {{
        put(1, "Field Tag 1");
        put(2, "Field Tag 2");
        // Add your game's official IDs here
    }};
}
