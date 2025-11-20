package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    public void drive(double forward, double strafe, double turn) {
        double leftFrontPower  = forward + strafe + turn;
        double rightFrontPower = forward - strafe - turn;
        double leftBackPower   = forward - strafe + turn;
        double rightBackPower  = forward + strafe - turn;

        setDrivePowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);
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
