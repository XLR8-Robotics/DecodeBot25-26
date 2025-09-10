package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.DcMotorConfig; // Added import
import org.firstinspires.ftc.teamcode.config.LimelightConfig; // Added import
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.Locale;

@TeleOp(name = "Limelight Viewer", group = "Vision")
public class LimelightViewer extends OpMode {
    private Robot robot;

    // String constants for motor names removed

    @Override
    public void init() {
        // Updated to use DcMotorConfig and LimelightConfig enums
        robot = new Robot(hardwareMap, 
                        DcMotorConfig.LEFT_FRONT, 
                        DcMotorConfig.LEFT_REAR, 
                        DcMotorConfig.RIGHT_FRONT, 
                        DcMotorConfig.RIGHT_REAR,
                        LimelightConfig.LIMELIGHT_DEFAULT);
    }

    @Override
    public void loop() {
        // robot.getLimelight().updateResult(); // This is now called in robot.teleopLoop() or robot.autonomousLoop()
        // If this OpMode doesn't call robot.teleopLoop(), then updateResult() must be called here:
        if (robot != null && robot.getLimelight() != null) {
             robot.getLimelight().updateResult();
        }

        assert robot != null;
        telemetry.addData("Has Targets", robot.getLimelight().hasTargets());
        telemetry.addData("Best Tag", robot.getLimelight().getBestTagName());
        telemetry.addData("Tag ID", robot.getLimelight().getBestTagId());
        telemetry.addData("tx", String.format(Locale.US, "%.2f", robot.getLimelight().getTx()));
        telemetry.addData("ty", String.format(Locale.US, "%.2f", robot.getLimelight().getTy()));
        
        Pose3D pose = robot.getLimelight().getBotPose();
        if (pose != null) {
            telemetry.addData("BotPose X (m)", String.format(Locale.US, "%.2f", pose.getPosition().x));
            telemetry.addData("BotPose Y (m)", String.format(Locale.US, "%.2f", pose.getPosition().y));
            telemetry.addData("BotPose Z (m)", String.format(Locale.US, "%.2f", pose.getPosition().z));
        } else {
            telemetry.addData("BotPose X (m)", "N/A");
            telemetry.addData("BotPose Y (m)", "N/A");
            telemetry.addData("BotPose Z (m)", "N/A");
        }
        telemetry.update(); // Ensure telemetry is updated
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stopRobot(); // Call stopRobot to stop Limelight and other subsystems
        }
    }
}
