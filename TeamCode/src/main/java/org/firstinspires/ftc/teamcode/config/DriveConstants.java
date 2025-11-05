package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import java.util.Arrays;

/**
 * RoadRunner Drive Constants
 * 
 * These constants define the physical characteristics and limits of your robot's drivetrain.
 * Tune these values to match your robot's actual performance.
 * 
 * For tuning instructions, see: https://learnroadrunner.com/
 */
@Config
public class DriveConstants {

    /*
     * These are motor constants that should be listed online for your motors
     */
    public static final double TICKS_PER_REV = 537.7;
    public static final double MAX_RPM = 312;

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., wheel encoders).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    public static final boolean RUN_USING_ENCODER = true;
    public static double MOTOR_VELO_PID = 0.5;
    public static double MOTOR_VELO_PID2 = 0.0;
    public static double MOTOR_VELO_PID3 = 0.0;

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; see https://docs.google.com/document/d/1tyWrDf_wbNeTNzQkP6hG7ZHpNmt58PlfYwlcFOSZdVg/edit).
     */
    public static double WHEEL_RADIUS = 2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 13.5; // in

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static double kV = 0.017;
    public static double kA = 0.003;
    public static double kStatic = 0.1;

    /*
     * These values are used to generate the trajectories for your robot. To ensure proper operation,
     * the constraints should never exceed @link{MAX_VEL} and @link{MAX_ACCEL}.
     * The maximum values are based on the experimental results from @link{RobotDriveEncoderCalibrationOpMode}.
     * 
     * Note that the maximum velocity and acceleration are limited by the motor's maximum RPM and the
     * wheel radius.
     */
    public static double MAX_VEL = 30;
    public static double MAX_ACCEL = 30;
    public static double MAX_ANG_VEL = Math.toRadians(180);
    public static double MAX_ANG_ACCEL = Math.toRadians(180);

    /*
     * Adjust the orientations here to match your robot. See the FTC Coordinate system convention.
     */
    public static double LATERAL_MULTIPLIER = 1.0;

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMaxRpm() {
        return MAX_RPM * GEAR_RATIO;
    }

    /*
     * These are the trajectory constraints for RoadRunner.
     * You can create custom constraints by combining these or creating your own.
     */
    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}

