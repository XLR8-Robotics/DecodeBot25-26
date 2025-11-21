package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
@TeleOp(name = "LiftServoTester", group = "Tester")

public class LiftServoTester extends LinearOpMode {
    Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            intake.update(gamepad1);

        }

    }

}
