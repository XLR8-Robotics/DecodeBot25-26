package org.firstinspires.ftc.teamcode.config;

/**
 * This class is a centralized configuration file for the robot.
 * It contains nested classes for different categories of constants, such as
 * hardware names, motor speeds, and servo positions.
 * This makes it easy to adjust any robot parameter from one location.
 */
public class Constants {

    /**
     * Contains all hardware device names.
     */
    public static class HardwareConfig {
        // Drivetrain Motors
        public static final String DRIVE_MOTOR_LEFT_FRONT = "leftFrontMotor";
        public static final String DRIVE_MOTOR_RIGHT_FRONT = "rightFrontMotor";
        public static final String DRIVE_MOTOR_LEFT_REAR = "leftRearMotor";
        public static final String DRIVE_MOTOR_RIGHT_REAR = "rightRearMotor";

        // Subsystem Motors & Servos
        public static final String TURRET_MOTOR = "turretMotor";
        public static final String INTAKE_MOTOR = "intakeMotor";
        public static final String SHOOTER_MOTOR = "shooterMotor";
        public static final String HOOD_SERVO = "hoodServo";
        public static final String LIMELIGHT_NAME = "limelight";

        // Odometry Pods
        public static final String LEFT_ENCODER = "left_encoder";
        public static final String RIGHT_ENCODER = "right_encoder";
        public static final String CENTER_ENCODER = "center_encoder";
    }

    /**
     * Contains constants for the Turret subsystem.
     */
    public static class TurretConfig {
        // Speed for turret rotation (0.0 to 1.0)
        public static final double TURRET_SPEED = 0.6;
    }

    /**
     * Contains constants for the Intake subsystem.
     */
    public static class IntakeConfig {
        // Speed for the intake motor (0.0 to 1.0)
        public static final double INTAKE_SPEED = 0.8;
    }

    /**
     * Contains constants for the Shooter subsystem.
     */
    public static class ShooterConfig {
        // Speed for the shooter wheel (0.0 to 1.0)
        public static final double SHOOTER_SPEED = 0.9;

        // Servo positions for the shooter hood (0.0 to 1.0)
        public static final double HOOD_DEFAULT_POSITION = 0.5;
        public static final double HOOD_UP_POSITION = 0.75;
        public static final double HOOD_DOWN_POSITION = 0.25;
    }

    /**
     * Contains constants for Odometry
     */
    public static class OdometryConfig {
        public static final double ODOMETRY_TRACK_WIDTH = 13.5;
        public static final double ODOMETRY_FORWARD_OFFSET = 5.5;
        public static final double ODOMETRY_WHEEL_DIAMETER = 2.0;
        public static final double ODOMETRY_TICKS_PER_REV = 8192;
    }

    /**
     * Contains constants for the Limelight subsystem
     */
    public static class LimelightConfig {
        public static final double LIMELIGHT_HEIGHT = 20.0;
        public static final double LIMELIGHT_ANGLE = 25.0;
    }
}
