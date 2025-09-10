package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TuningConfig;
import org.firstinspires.ftc.teamcode.config.DcMotorConfig;
import org.firstinspires.ftc.teamcode.config.OdometryConfig;
import org.firstinspires.ftc.teamcode.config.LimelightConfig; // Added import
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsHelper;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "Basic Drive", group = "Drive")
public class BasicDrive extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap,
                        DcMotorConfig.LEFT_FRONT,
                        DcMotorConfig.LEFT_REAR,
                        DcMotorConfig.RIGHT_FRONT,
                        DcMotorConfig.RIGHT_REAR,
                        LimelightConfig.LIMELIGHT_DEFAULT); // Added LimelightConfig

        // OPTIONAL (enable if using goBILDA Pinpoint):
        // Updated to use OdometryConfig enum
        try {
            robot.configureOdometry(OdometryConfig.PINPOINT);
        } catch (Exception ignored) {}
    }

    @Override
    public void start() {
        robot.startTeleOp();
        PanelsHelper.init();
    }

    @Override
    public void loop() {
        // normalScale, slowScale come from TuningConfig
        robot.teleopLoop(gamepad1, TuningConfig.TELEOP_NORMAL_SCALE, TuningConfig.TELEOP_SLOW_SCALE);
        telemetry.addData("Pose X", robot.getDriveTrain().getPose().getX());
        telemetry.addData("Pose Y", robot.getDriveTrain().getPose().getY());
        telemetry.addData("Heading", robot.getDriveTrain().getPose().getHeading());
        PanelsHelper.drawDebug(robot.getDriveTrain().getFollower());

        // Example of how to get Limelight data in your OpMode
        if (robot.getLimelight() != null && robot.getLimelight().hasTargets()) {
            telemetry.addData("Limelight Tag ID", robot.getLimelight().getBestTagId());
            telemetry.addData("Limelight Tag Name", robot.getLimelight().getBestTagName());
            telemetry.addData("Limelight TX", "%.2f", robot.getLimelight().getTx());
            telemetry.addData("Limelight TY", "%.2f", robot.getLimelight().getTy());
        }
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stopRobot(); // Call stopRobot to stop Limelight and other subsystems
        }
    }
}
