package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TuningConfig;
import org.firstinspires.ftc.teamcode.Autos.commands.*;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsHelper;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "Square Auto (Commands)", group = "Drive")
public class SquareAuto extends LinearOpMode {
    private static final String LF = "leftFront";
    private static final String LR = "leftRear";
    private static final String RF = "rightFront";
    private static final String RR = "rightRear";

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, LF, LR, RF, RR);

        try {
            robot.configureOdometry(
                    TuningConfig.PINPOINT_DEVICE_NAME,
                    TuningConfig.PINPOINT_X_OFFSET_MM,
                    TuningConfig.PINPOINT_Y_OFFSET_MM,
                    TuningConfig.PINPOINT_FORWARD_ENCODER_FORWARD,
                    TuningConfig.PINPOINT_STRAFE_ENCODER_FORWARD,
                    TuningConfig.PINPOINT_USE_FOUR_BAR_POD
            );
        } catch (Exception ignored) {}

        double d = TuningConfig.AUTO_SQUARE_SIDE_INCHES;

        AutoCommand square = new Sequential(
                new DriveDeltaX(d),
                new DriveDeltaY(d),
                new DriveDeltaX(-d),
                new DriveDeltaY(-d)
        );

        waitForStart();
        if (isStopRequested()) return;

        square.init(robot);
        while (opModeIsActive() && !square.isFinished(robot)) {
            square.execute(robot);
            telemetry.addData("X", robot.getDriveTrain().getPose().getX());
            telemetry.addData("Y", robot.getDriveTrain().getPose().getY());
            telemetry.update();
            PanelsHelper.drawDebug(robot.getDriveTrain().getFollower());
        }
        square.end(robot);
    }
}


