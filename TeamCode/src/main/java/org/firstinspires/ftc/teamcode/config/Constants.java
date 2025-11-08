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
        public static final String SHOOTER_BLOCKER = "shooterBlocker";
        public static final String INTAKE_MOTOR = "intakeMotor";
        public static final String LIFT_SERVO = "liftServo";
        public static final String SHOOTER_MOTOR = "shooterMotor";
        public static final String HOOD_SERVO = "hoodServo";
        public static final String LIMELIGHT_NAME = "limelight";
        public static final String PINPOINT_DEVICE_NAME = "pinpoint"; // REQUIRED if using Pinpoint

        // Sensors
        public static final String TURRET_LIMIT_LEFT = "turret_limit_left";
        public static final String TURRET_LIMIT_RIGHT = "turret_limit_right";
        public static final String INTAKE_DISTANCE_LEFT = "intake_distance_left";
        public static final String INTAKE_DISTANCE_RIGHT = "intake_distance_right";

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
        // Servo positions for shooterBlocker Servo
        public static final double SHOOTER_BLOCKER_ZERO_POSITION= 0.0;
        public static final double SHOOTER_BLOCKER_BLOCKING_POSITION = 0.45;
    }

    /**
     * Contains constants for automating the turret aiming.
     */
    public static class TurretAimingConfig {
        // Proportional gain for the aiming controller. Tune this value carefully.
        public static final double AIMING_KP = 0.03;

        // Oscillation settings for when no target is found
        public static final double OSCILLATION_SPEED = 0.3;  // Speed of oscillation (0.0 to 1.0)
        public static final double OSCILLATION_PERIOD_MS = 3000.0;  // Time for one full oscillation cycle (ms)
        public static final boolean ENABLE_OSCILLATION = true;  // Enable/disable oscillation feature
    }
    /**
     * Contains constants for automating the shooter aiming.
     */
    public static class ShooterAimingConfig {
        // Proportional gain for the aiming controller. Tune this value carefully.
        public static final double HOOD_KP = 0.01;
        public static final double POWER_KP = 0.02;
    }


    /**
     * Contains constants for the Intake subsystem.
     */
    public static class IntakeConfig {
        // Speed for the intake motor (0.0 to 1.0)
        public static final double INTAKE_SPEED = 0.8;
        public static final double LIFT_SERVO_NOT_LIFTING_POSITION = 0.0;
        public static final double LIFT_SERVO_LIFTING_POSITION = 0.65;
        // Distance threshold in centimeters for detecting an object
        public static final double INTAKE_DISTANCE_THRESHOLD_CM = 5.0;

    }

    /**
     * Contains constants for the Shooter subsystem.
     */
    public static class ShooterConfig {
        // Speed for the shooter wheel (0.0 to 1.0)
        public static final double SHOOTER_SPEED = 0.9;
        //SERVOs
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
        public static final double APRIL_TAG_HEIGHT = 6.5;
    }
     /**
     * Contains constants for the automated launch sequence.
     */
    public static class LaunchSequenceConfig {
        // Time in milliseconds for the shooter to spin up to speed.
        public static final long SHOOTER_SPIN_UP_TIME_MS = 2000; // 2 seconds

        // Time in milliseconds for the intake to run in reverse when cancelled.
        public static final long INTAKE_REVERSE_TIME_MS = 1000; // 1 second

        // The time window in milliseconds to detect a triple press of the cancel button.
        public static final long TRIPLE_PRESS_TIMEOUT_MS = 500; // 0.5 seconds
    }

}
