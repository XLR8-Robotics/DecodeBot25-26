package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TuningConfig;
import org.firstinspires.ftc.teamcode.Autos.commands.*;
import org.firstinspires.ftc.teamcode.config.OdometryConfig; // Added import
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsHelper;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "Forward+Strafe Auto (Commands)", group = "Drive")
public class ForwardStrafeAuto extends LinearOpMode {
    private static final String LF = "leftFront";
    private static final String LR = "leftRear";
    private static final String RF = "rightFront";
    private static final String RR = "rightRear";

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, LF, LR, RF, RR);

        try {
            // Updated to use OdometryConfig.PINPOINT
            robot.configureOdometry(OdometryConfig.PINPOINT);
        } catch (Exception ignored) {}

        AutoCommand routine = new Sequential(
                new DriveDeltaX(TuningConfig.AUTO_FORWARD_INCHES),
                new DriveDeltaY(TuningConfig.AUTO_SQUARE_SIDE_INCHES)
        );

        waitForStart();
        if (isStopRequested()) return;

        routine.init(robot);
        while (opModeIsActive() && !routine.isFinished(robot)) {
            routine.execute(robot);
            telemetry.addData("X", robot.getDriveTrain().getPose().getX());
            telemetry.addData("Y", robot.getDriveTrain().getPose().getY());
            telemetry.update();
            PanelsHelper.drawDebug(robot.getDriveTrain().getFollower());
        }
        routine.end(robot);
    }
}

