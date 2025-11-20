package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.Constants;

public class BasicDriveTrain {
    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public BasicDriveTrain(HardwareMap hardwareMap) {

        leftFront = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.DRIVE_MOTOR_LEFT_FRONT);
        leftBack = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.DRIVE_MOTOR_LEFT_REAR);
        rightBack = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.DRIVE_MOTOR_RIGHT_REAR);
        rightFront = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.DRIVE_MOTOR_RIGHT_FRONT);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setDrivePowers(double leftFrontPower, double leftBackPower, double rightBackPower, double rightFrontPower) {
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);
    }


    public void drive(double drive, double strafe, double rotate, double speedModifier) {
        // Ensure that joystick inputs are within the correct range (-1.0 to 1.0)
        drive = Range.clip(drive, -1.0, 1.0);
        strafe = Range.clip(strafe, -1.0, 1.0);
        rotate = Range.clip(rotate, -1.0, 1.0);

        // Apply the mecanum drive formula to calculate the power for each motor
        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower = drive - strafe + rotate;
        double backRightPower = drive + strafe - rotate;

        // Normalize the values to prevent them from exceeding the motor's maximum power
        double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        leftFront.setPower(frontLeftPower * speedModifier);
        rightFront.setPower(frontRightPower * speedModifier);
        leftBack.setPower(backLeftPower * speedModifier);
        rightBack.setPower(backRightPower * speedModifier);
    }
    /**
     * Returns the current power of the drivetrain motors.
     * @return An array of doubles containing the power of each motor in the order: [leftFront, rightFront, leftBack, rightBack]
     */
    public double[] getMotorPowers() {
        return new double[]{
            leftFront.getPower(),
            rightFront.getPower(),
            leftBack.getPower(),
            rightBack.getPower()
        };
    }
}
