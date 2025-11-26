package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.Constants.HardwareConfig;


public class Constants {


    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.21);
            //.forwardZeroPowerAcceleration(-53.4) //TODO test forwardzeropoweraccelerationtuner
            //.lateralZeroPowerAcceleration(-74.7) //TODO test lateralzeropoweraccelerationtuner
            //.translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.01, 0)) //TODO test panels translation
           // .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.01, 0)) //TODO test panels heading
            //.drivePIDFCoefficients(new FilteredPIDFCoefficients(0.003,0.0,0.005,0.6,0.0)) //TODO test panels drive
            //.centripetalScaling(0.005); //TODO test panels centripetal

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(.5)
            .rightFrontMotorName(HardwareConfig.DRIVE_MOTOR_RIGHT_FRONT)
            .rightRearMotorName(HardwareConfig.DRIVE_MOTOR_RIGHT_REAR)
            .leftRearMotorName(HardwareConfig.DRIVE_MOTOR_LEFT_REAR)
            .leftFrontMotorName(HardwareConfig.DRIVE_MOTOR_LEFT_FRONT)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
            //.rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            //.yVelocity()
            //.xVelocity()
            //.forwardZeroPowerAcceleration()
            //.lateralZeroPowerAcceleration(deceleration);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5.75)
            .strafePodX(6.25)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(HardwareConfig.PINPOINT_DEVICE_NAME)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();




    }

}