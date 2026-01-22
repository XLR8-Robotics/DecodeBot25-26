package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

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
        public static final String DRIVE_MOTOR_LEFT_FRONT = "leftFront";
        public static final String DRIVE_MOTOR_RIGHT_FRONT = "rightFront";
        public static final String DRIVE_MOTOR_LEFT_REAR = "leftRear";
        public static final String DRIVE_MOTOR_RIGHT_REAR = "rightRear";

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
        public static final String TURRET_LIMIT_LEFT = "turretLimitLeft";
        public static final String TURRET_LIMIT_RIGHT = "turretLimitRight";
        public static final String INTAKE_DISTANCE_LEFT = "intakeLeft";
        public static final String INTAKE_DISTANCE_RIGHT = "intakeRight";

        // Odometry Pods
        public static final String LEFT_ENCODER = "left_encoder";
        public static final String RIGHT_ENCODER = "right_encoder";
        public static final String CENTER_ENCODER = "center_encoder";
    }

    /**
     * Contains constants for the Drivetrain.
     */
    public static class DrivetrainConfig {
        public static final double DRIVE_SPEED_MULTIPLIER = 0.8; // Adjust this value to change the robot's overall speed
    }

    /**
     * Contains constants for the Turret subsystem.
     */
    public static class TurretConfig {
        // Speed for turret rotation (0.0 to 1.0)
        public static final double TURRET_SPEED = 0.4;
        // Servo positions for shooterBlocker Servo
        public static final double SHOOTER_BLOCKER_ZERO_POSITION= 0.45;
        public static final double SHOOTER_BLOCKER_BLOCKING_POSITION = 0.0;

        // Encoder and Gear Ratio for Turret Angle Calculation
        // This is for a goBILDA 5202 series motor. Change if you use a different motor.
        public static final double TURRET_TICKS_PER_REV = 537.7;
        // This is the gear ratio between the motor and the turret.
        // Example: If the motor spins 4 times for every 1 turret rotation, the ratio is 4.0
        public static final double TURRET_GEAR_RATIO = 1.0; // CHANGE THIS to your robot's gear ratio
    }

    /**
     * Contains constants for automating the turret aiming.
     */
    @Config
    public static class TurretAimingConfig {
        // Legacy Software PID gains
        public static double AIMING_KP = 0.03;
        public static double AIMING_KI = 0.0; // Tune this to eliminate steady-state error
        public static double AIMING_KD = 0.0; // Tune this to reduce oscillation

        // Hardware PID for Auto-Aim (RUN_TO_POSITION)
        public static double HARDWARE_P = 15.0;
        public static double HARDWARE_I = 0.0;
        public static double HARDWARE_D = 0.0;
        public static double HARDWARE_F = 0.0;

        // Search settings for when target is lost
        public static double SEARCH_POWER = 0.3; // Slower power for search movements
        public static double SEARCH_DWELL_TIME = 0.3; // Time to wait at each position (seconds)
        public static double TARGET_LOST_TIMEOUT = 2.0;
        public static double SEARCH_INITIAL_ANGLE = 15.0; // Initial search sweep angle
        public static double SEARCH_ANGLE_INCREMENT = 20.0; // Angle increment per sweep
        public static double SEARCH_MAX_ANGLE = 135.0; // Maximum search sweep angle
        public static boolean IMU_SEARCH_ENABLED = true; // Enable/disable IMU-based search when target is lost

        // Oscillation settings for when no target is found (Legacy - kept if needed but search params above are preferred)
        public static double OSCILLATION_SPEED = 0.3;  // Speed of oscillation (0.0 to 1.0)
        public static double OSCILLATION_PERIOD_MS = 3000.0;  // Time for one full oscillation cycle (ms)
        public static boolean ENABLE_OSCILLATION = true;  // Enable/disable oscillation feature
    }
    /**
     * Contains constants for automating the shooter aiming.
     */
    @Config
    public static class ShooterAimingConfig {
        // Proportional gain for the aiming controller. Tune this value carefully.
        public static double HOOD_KP = 0.01;
        public static double POWER_KP = 0.02;
    }


    /**
     * Contains constants for the Intake subsystem.
     */
    public static class IntakeConfig {
        // Speed for the intake motor (0.0 to 1.0)
        public static final double INTAKE_SPEED = 0.8;
        public static final double LIFT_SERVO_NOT_LIFTING_POSITION = 0.05;
        public static final double LIFT_SERVO_LIFTING_POSITION = 0.75;
        // Distance threshold in centimeters for detecting an object
        public static final double INTAKE_DISTANCE_THRESHOLD_CM = 5.0;

    }

    /**
     * Contains constants for the Shooter subsystem.
     */
    @Config
    public static class ShooterConfig {
        // Shooter wheel speeds (0.0 to 1.0) - better naming
        
        //public static double SHOOTER_RPM_NEAR = 2400;
        public static double SHOOTER_RPM_NEAR = 2500;
        public static double SHOOTER_RPM_NEAR_BLUE = 2575;
        public static double HOOD_POSITION_NEAR = 0.25;
        
        public static double SHOOTER_RPM_MEDIUM = 2650;
        public static double HOOD_POSITION_MEDIUM = 0.5;
        
        public static double SHOOTER_RPM_FAR = 3150;

        public static double SHOOTER_RPM_FAR_AUTO = 3210;
        public static double SHOOTER_RPM_FAR_AUTO_RED = 3200;
        public static double HOOD_POSITION_FAR = 0.25;
        public static double SHOOTER_RPM_IDLE = 1500;
        public static double HOOD_POSITION_FAR_AUTO = 0.5;
        
        
    }

    /**
     * Contains constants for automatic distance-based shooting.
     * Tune these values based on your robot's shooting performance.
     * Now uses RPM-based control for consistent, battery-independent shooting.
     */
    @Config
    public static class AutoShootingConfig {
        // Motor specifications for RPM calculation
        // IMPORTANT: Update this to match your actual shooter motor!
        // Common FTC motors:
        // - goBILDA 5202/5203: 537.7 ticks/rev
        // - REV HD Hex Motor: 28 ticks/rev  
        // - AndyMark NeveRest: 1120 ticks/rev (20:1), 560 ticks/rev (40:1)
        public static double SHOOTER_MOTOR_TICKS_PER_REV = 28.0; // goBILDA 5000 series 1:1
        
        // Distance ranges in inches for different shooting zones
        public static double MIN_SHOOTING_DISTANCE = 12.0; // Minimum safe shooting distance
        public static double MAX_SHOOTING_DISTANCE = 120.0; // Maximum effective shooting distance
        
        // Distance-based lookup table breakpoints
        // These are the distance breakpoints where shooting parameters change
        public static double[] DISTANCE_BREAKPOINTS = {
            12.0,   // Very close shots
            24.0,   // Close shots
            36.0,   // Medium-close shots
            48.0,   // Medium shots
            60.0,   // Medium-far shots
            72.0,   // Far shots
            84.0,   // Very far shots
            96.0,   // Extra far shots
            120.0   // Maximum range shots
        };
        
        // RPM-based shooter speeds for each distance breakpoint
        // These values are more consistent than power percentages!
        // TUNE THESE VALUES based on your robot's actual performance
        public static double[] SHOOTER_RPM_VALUES = {
            2000,   // Very close - low RPM for gentle shots
            2300,   // Close - slightly higher RPM
            2600,   // Medium-close
            2900,   // Medium
            3200,   // Medium-far
            3500,   // Far
            3800,   // Very far
            4100,   // Extra far
            4400    // Maximum range - high RPM for power shots
        };
        
        // Corresponding hood position values for each distance breakpoint
        public static double[] HOOD_POSITION_VALUES = {
            0.25,   // Very close - low arc (HOOD_MIN)
            0.31,   // Close - slightly higher arc
            0.37,   // Medium-close (HOOD_POS_3)
            0.44,   // Medium (HOOD_POS_4)
            0.50,   // Medium-far (HOOD_CENTER)
            0.56,   // Far (HOOD_POS_6)
            0.62,   // Very far (HOOD_POS_7)
            0.69,   // Extra far (HOOD_POS_8)
            0.75    // Maximum range - high arc (HOOD_MAX)
        };
        
        // Control mode selection
        // public static final boolean USE_RPM_CONTROL = true; // Removed since RPM control is now mandatory
        public static boolean ENABLE_INTERPOLATION = true; // Enable smooth interpolation between breakpoints
        
        // Safety limits for RPM control
        public static double MIN_SHOOTER_RPM = 1500;  // Minimum safe RPM
        public static double MAX_SHOOTER_RPM = 6000;  // Maximum safe RPM for 1:1 motor
        
        // Fallback safety limits for power control (if RPM is disabled)
        public static double MIN_SHOOTER_POWER = 0.60;
        public static double MAX_SHOOTER_POWER = 1.0;
        public static double MIN_HOOD_POSITION = 0.25;
        public static double MAX_HOOD_POSITION = 0.75;
        
        // Velocity control PID gains (for RPM control)
        // These may need tuning based on your motor and load
        public static double VELOCITY_KP = 0.0;  // Proportional gain
        public static double VELOCITY_KI = 0.0;   // Integral gain
        public static double VELOCITY_KD = 0.0;   // Derivative gain
        public static double VELOCITY_KF = 0.0;  // Feed-forward gain
        
        // Auto-shooting timeouts and delays
        public static long DISTANCE_MEASUREMENT_TIMEOUT_MS = 100; // Time to wait for stable distance reading
        public static long PARAMETER_UPDATE_DELAY_MS = 50; // Delay between parameter updates
        public static long RPM_STABILIZATION_TIME_MS = 500; // Time to wait for RPM to stabilize
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
        public static final double LIMELIGHT_HEIGHT = 3.3599;
        public static final double LIMELIGHT_ANGLE = 16.5;
        public static final double APRIL_TAG_HEIGHT = 30.0;
    }
     /**
     * Contains constants for the automated launch sequence.
     */
    @Config
    public static class LaunchSequenceConfig {
        // Time in milliseconds for the shooter to spin up to speed.
        public static long SHOOTER_SPIN_UP_TIME_MS = 800; // 2 seconds
         
         public static double SHOOTER_LAUNCH_RPM = 3500; 

        // Time in milliseconds for the intake to run in reverse when cancelled.
        public static long INTAKE_REVERSE_TIME_MS = 1500; // 1 second

        // The time window in milliseconds to detect a triple press of the cancel button.
        public static long TRIPLE_PRESS_TIMEOUT_MS = 500; // 0.5 seconds
        
        // Time window for single cross button press detection
        public static long SINGLE_PRESS_TIMEOUT_MS = 200;
        
        // Time for lift servo to stay in lifting position during sequence
        public static long LIFT_SERVO_HOLD_TIME_MS = 400;
        
        // Number of cross button presses needed to cancel launch sequence
        public static int TRIPLE_PRESS_COUNT = 3;
    }

}
