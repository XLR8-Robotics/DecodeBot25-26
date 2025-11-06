package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.Constants;

/**
 * Subsystem for controlling the robot's drivetrain.
 */
public class Drivetrain {
    private final DcMotorEx leftFront;
    private final DcMotorEx rightFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightRear;

    public Drivetrain(HardwareMap hardwareMap) {
        // Initialize motors from the hardware map using names from the Constants file
        leftFront = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.DRIVE_MOTOR_LEFT_FRONT);
        rightFront = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.DRIVE_MOTOR_RIGHT_FRONT);
        leftRear = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.DRIVE_MOTOR_LEFT_REAR);
        rightRear = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.DRIVE_MOTOR_RIGHT_REAR);

        // Reverse the right-side motors for mecanum drive
        // You may need to change this depending on your robot's motor orientation
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Controls the drivetrain using mecanum drive kinematics from joystick inputs.
     * @param forward The forward/backward input (typically from a y-axis joystick)
     * @param strafe The strafing input (typically from an x-axis joystick)
     * @param turn The turning input (typically from another x-axis joystick)
     */
    public void manualDrive(double forward, double strafe, double turn) {
        // Mecanum drive kinematics
        double leftFrontPower = forward + strafe + turn;
        double rightFrontPower = forward - strafe - turn;
        double leftRearPower = forward - strafe + turn;
        double rightRearPower = forward + strafe - turn;

        // Normalize the motor powers to ensure they are within the -1.0 to 1.0 range
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftRearPower));
        max = Math.max(max, Math.abs(rightRearPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftRearPower /= max;
            rightRearPower /= max;
        }

        setMotorPowers(leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
    }

    /**
     * Sets the power for each of the four drivetrain motors directly.
     */
    private void setMotorPowers(double lfPower, double rfPower, double lrPower, double rrPower) {
        leftFront.setPower(lfPower);
        rightFront.setPower(rfPower);
        leftRear.setPower(lrPower);
        rightRear.setPower(rrPower);
    }

    /**
     * Returns the current power of all four drivetrain motors.
     * @return An array of motor powers: [leftFront, rightFront, leftRear, rightRear]
     */
    public double[] getMotorPowers(){
        return new double[]{leftFront.getPower(), rightFront.getPower(), leftRear.getPower(), rightRear.getPower()};
    }
}
