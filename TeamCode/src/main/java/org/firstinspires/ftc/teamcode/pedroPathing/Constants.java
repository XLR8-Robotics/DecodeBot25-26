package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.localization.ThreeWheelLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

// This is the Constants file for the pedroPathing library.
// This is being modified to use the centralized Constants file from the main project.
import org.firstinspires.ftc.teamcode.config.Constants.HardwareConfig;
import org.firstinspires.ftc.teamcode.config.Constants.OdometryConfig;

public class Constants {
    public static FollowerConstants followerConstants;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        // Create a new FollowerConstants object
        followerConstants = new FollowerConstants();

        // Set the odometry constants from the main Constants file
        followerConstants.setTrackWidth(OdometryConfig.ODOMETRY_TRACK_WIDTH);
        followerConstants.setForwardOffset(OdometryConfig.ODOMETRY_FORWARD_OFFSET);

        // Create the ThreeWheelLocalizer with the encoder names from the main Constants file
        ThreeWheelLocalizer localizer = new ThreeWheelLocalizer(
                hardwareMap,
                OdometryConfig.ODOMETRY_TICKS_PER_REV,
                OdometryConfig.ODOMETRY_WHEEL_DIAMETER,
                HardwareConfig.LEFT_ENCODER,
                HardwareConfig.RIGHT_ENCODER,
                HardwareConfig.CENTER_ENCODER
        );

        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .setLocalizer(localizer) // Set our custom localizer
                .build();
    }
}
