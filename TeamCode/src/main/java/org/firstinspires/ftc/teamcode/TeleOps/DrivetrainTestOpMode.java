package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BasicDriveTrain;

@TeleOp(name = "Drivetrain Test", group = "Testing")
public class DrivetrainTestOpMode extends LinearOpMode {

    private BasicDriveTrain basicDriveTrain;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drivetrain
        basicDriveTrain = new BasicDriveTrain(hardwareMap);

        telemetry.addData("Status", "Initialized and Ready to Test Drivetrain!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update drivetrain control
            updateDrivetrainControl(gamepad1);

            // --- Telemetry ---
            displayTelemetry();
        }
    }

    /**
     * Updates drivetrain control based on gamepad input.
     * @param gamepad The gamepad to read input from
     */
    private void updateDrivetrainControl(Gamepad gamepad) {
        // Standard mecanum drive control: left stick for translation, right stick for rotation
        double forward = gamepad.left_stick_y * Constants.DrivetrainConfig.DRIVE_SPEED_MULTIPLIER;
        double strafe = gamepad.right_stick_x * Constants.DrivetrainConfig.DRIVE_SPEED_MULTIPLIER;
        double turn = gamepad.left_stick_x * Constants.DrivetrainConfig.DRIVE_SPEED_MULTIPLIER;
        basicDriveTrain.drive(-forward, -strafe, turn);
    }

    private void displayTelemetry() {
        // Display the power of each drivetrain motor
        double[] motorPowers = basicDriveTrain.getMotorPowers();
        telemetry.addData("Front Left Power", "%.2f", motorPowers[0]);
        telemetry.addData("Front Right Power", "%.2f", motorPowers[1]);
        telemetry.addData("Back Left Power", "%.2f", motorPowers[2]);
        telemetry.addData("Back Right Power", "%.2f", motorPowers[3]);

        telemetry.update();
    }
}
