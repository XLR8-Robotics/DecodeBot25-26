package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TuningConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsHelper;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "Basic Drive", group = "Drive")
public class BasicDrive extends OpMode {
    private Robot robot;

    // REQUIRED: change these to match your RC configuration motor names
    private static final String LF = "leftFront";
    private static final String LR = "leftRear";
    private static final String RF = "rightFront";
    private static final String RR = "rightRear";

    @Override
    public void init() {
        robot = new Robot(hardwareMap, LF, LR, RF, RR);

        // OPTIONAL (enable if using goBILDA Pinpoint):
        // - device name must match RC config (commonly "pinpoint")
        // - offsets are in millimeters: x = left(+)/right(-) of center, y = forward(+)/back(-) of center
        // - encoder directions must match your pod directions
        try {
            // EXAMPLE OFFSETS ONLY — UPDATE FOR YOUR ROBOT
            robot.configureOdometry("pinpoint", -84.0, -168.0, true, true, true);
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
    }
}


