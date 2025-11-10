package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "Shooting Sequence Test", group = "Test")
public class ShootingSequenceTest extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot();
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press (Cross) to start the shooting sequence.");
        telemetry.addData(">", "Press (Circle) to cancel the shooting sequence.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Start the launch sequence when 'Cross' is pressed
            if (gamepad1.cross) {
                robot.startLaunchSequence();
            }

            // Cancel the launch sequence when 'Circle' is pressed
            if (gamepad1.circle) {
                robot.cancelLaunchSequence();
            }

            // Update the robot's state machine
            robot.updateLaunchSequence();


            // Telemetry
            telemetry.addData("Launch Sequence State", robot.getLaunchSequenceState());
            telemetry.addData("Shooter Power", "%.2f", robot.shooter.getMotorPower());
            telemetry.addData("Shooter Blocker Position", "%.2f", robot.turret.getShooterBlockerPosition());
            telemetry.addData("Intake Power", "%.2f", robot.intake.getMotorPower());
            telemetry.addData("Object Detected", robot.intake.isObjectDetected());
            telemetry.update();
        }
    }
}
