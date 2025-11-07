package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.Constants;

/**
 * Subsystem for controlling the intake mechanism.
 */
public class Intake {
    private final DcMotorEx intakeMotor;
    private final Servo liftServo;

    private boolean isLifted = false;
    private boolean previousCircleButtonState = false;

    public Intake(HardwareMap hardwareMap) {
        this.intakeMotor = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.INTAKE_MOTOR);
        this.liftServo = hardwareMap.get(Servo.class, Constants.HardwareConfig.LIFT_SERVO);
        // If the intake runs in the wrong direction, you can reverse it by uncommenting the next line.
        // this.intakeMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Call this method in your TeleOp loop to control the intake with the gamepad triggers.
     * @param gamepad The gamepad that will control the intake.
     */
    public void update(Gamepad gamepad) {
        // The right trigger controls intake, the left trigger controls outtake.
        double intakePower = gamepad.right_trigger - gamepad.left_trigger;

        intakeMotor.setPower(intakePower * Constants.IntakeConfig.INTAKE_SPEED);

        boolean currentCircleButtonState = gamepad.circle;
        if (currentCircleButtonState && !previousCircleButtonState) {
            isLifted = !isLifted;
            if (isLifted) {
                liftServo.setPosition(Constants.IntakeConfig.LIFT_SERVO_LIFTING_POSITION);
            } else {
                liftServo.setPosition(Constants.IntakeConfig.LIFT_SERVO_NOT_LIFTING_POSITION);
            }
        }
        previousCircleButtonState = currentCircleButtonState;
    }

    /**
     * Returns the current power of the intake motor. Useful for telemetry.
     * @return The current power of the intake motor.
     */
    public double getMotorPower() {
        return intakeMotor.getPower();
    }
    public double getLiftServoPosition(){return liftServo.getPosition();}
}
