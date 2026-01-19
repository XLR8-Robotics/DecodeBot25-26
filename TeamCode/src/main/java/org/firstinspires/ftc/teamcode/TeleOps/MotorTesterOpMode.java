package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.config.Constants.HardwareConfig;

/**
 * Motor Tester OpMode
 * 
 * This OpMode allows individual testing of each drive motor using the D-Pad.
 * 
 * Controls:
 * - D-Pad Up: Front Left Motor
 * - D-Pad Down: Back Left Motor
 * - D-Pad Left: Back Right Motor
 * - D-Pad Right: Front Right Motor
 */

@TeleOp(name = "Motor Tester", group = "Test")
public class MotorTesterOpMode extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors using constants
        try {
            leftFront = hardwareMap.get(DcMotor.class, HardwareConfig.DRIVE_MOTOR_LEFT_FRONT);
            rightFront = hardwareMap.get(DcMotor.class, HardwareConfig.DRIVE_MOTOR_RIGHT_FRONT);
            leftRear = hardwareMap.get(DcMotor.class, HardwareConfig.DRIVE_MOTOR_LEFT_REAR);
            rightRear = hardwareMap.get(DcMotor.class, HardwareConfig.DRIVE_MOTOR_RIGHT_REAR);

            // Set motor directions based on typical mecanum setup from Constants or default
            // You might need to adjust these depending on your specific wiring, 
            // but usually one side is reversed.
            // Referring to MecanumConstants in your project, leftFront is usually reversed.
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
            rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
            rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
            
            // Set zero power behavior
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        } catch (Exception e) {
            telemetry.addData("Error", "Motor initialization failed: " + e.getMessage());
            telemetry.update();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Use D-Pad to run motors:");
        telemetry.addData("D-Pad Up", "Front Left");
        telemetry.addData("D-Pad Down", "Back Left");
        telemetry.addData("D-Pad Left", "Back Right");
        telemetry.addData("D-Pad Right", "Front Right");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double power = 0.5; // Fixed power for testing

            // Front Left - D-Pad Up
            if (gamepad1.dpad_up) {
                leftFront.setPower(power);
                telemetry.addData("Active Motor", "Front Left");
            } else {
                leftFront.setPower(0);
            }

            // Back Left - D-Pad Down
            if (gamepad1.dpad_down) {
                leftRear.setPower(power);
                telemetry.addData("Active Motor", "Back Left");
            } else {
                leftRear.setPower(0);
            }

            // Back Right - D-Pad Left
            if (gamepad1.dpad_left) {
                rightRear.setPower(power);
                telemetry.addData("Active Motor", "Back Right");
            } else {
                rightRear.setPower(0);
            }

            // Front Right - D-Pad Right
            if (gamepad1.dpad_right) {
                rightFront.setPower(power);
                telemetry.addData("Active Motor", "Front Right");
            } else {
                rightFront.setPower(0);
            }

            // Telemetry for motor powers
            telemetry.addData("FL Power", leftFront.getPower());
            telemetry.addData("FR Power", rightFront.getPower());
            telemetry.addData("BL Power", leftRear.getPower());
            telemetry.addData("BR Power", rightRear.getPower());
            telemetry.update();
        }
    }
}
