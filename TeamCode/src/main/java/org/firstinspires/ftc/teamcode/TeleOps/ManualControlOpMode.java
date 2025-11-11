package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "Manual Control", group = "Iterative Opmode")
public class ManualControlOpMode extends OpMode {

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot();
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        robot.manualUpdate(gamepad1);
        telemetry.addData("Status", "Running");
    }

    @Override
    public void stop() {
        robot.stopAllMotors();
        telemetry.addData("Status", "Stopped");
    }
}
