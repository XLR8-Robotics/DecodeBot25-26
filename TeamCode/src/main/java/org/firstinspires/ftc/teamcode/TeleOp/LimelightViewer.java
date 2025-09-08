package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "Limelight Viewer", group = "Vision")
public class LimelightViewer extends OpMode {
    private Robot robot;

    private static final String LF = "leftFront";
    private static final String LR = "leftRear";
    private static final String RF = "rightFront";
    private static final String RR = "rightRear";

    @Override
    public void init() {
        robot = new Robot(hardwareMap, LF, LR, RF, RR);
    }

    @Override
    public void loop() {
        robot.getLimelight().update();
        telemetry.addData("Has Targets", robot.getLimelight().hasTargets());
        telemetry.addData("Best Tag", robot.getLimelight().getBestTagName());
        telemetry.addData("Tag ID", robot.getLimelight().getBestTagId());
        telemetry.addData("tx", robot.getLimelight().getTx());
        telemetry.addData("ty", robot.getLimelight().getTy());
        double[] pose = robot.getLimelight().getBotpose();
        telemetry.addData("botpose len", pose.length);
    }
}


