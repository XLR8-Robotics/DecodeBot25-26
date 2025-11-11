package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.Constants;

/**
 * Subsystem for controlling the intake mechanism.
 */
public class Intake {
    private final DcMotorEx intakeMotor;
    private final Servo liftServo;
    private final DistanceSensor leftDistanceSensor;
    private final DistanceSensor rightDistanceSensor;

    private boolean previousCircleButtonState = false;

    // State machine for the lift servo
    private enum LiftState {
        IDLE,
        LIFTING
    }
    private LiftState currentLiftState = LiftState.IDLE;
    private final ElapsedTime liftTimer = new ElapsedTime();


    public Intake(HardwareMap hardwareMap) {
        this.intakeMotor = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.INTAKE_MOTOR);
        this.liftServo = hardwareMap.get(Servo.class, Constants.HardwareConfig.LIFT_SERVO);
        this.leftDistanceSensor = hardwareMap.get(DistanceSensor.class, Constants.HardwareConfig.INTAKE_DISTANCE_LEFT);
        this.rightDistanceSensor = hardwareMap.get(DistanceSensor.class, Constants.HardwareConfig.INTAKE_DISTANCE_RIGHT);

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

        setPower(intakePower * Constants.IntakeConfig.INTAKE_SPEED);

        // --- Lift Servo Logic ---
        boolean currentCircleButtonState = gamepad.circle;

        // State machine for the lift servo
        switch (currentLiftState) {
            case IDLE:
                // On button press, start the lifting sequence
                if (currentCircleButtonState && !previousCircleButtonState) {
                    setLiftPosition(Constants.IntakeConfig.LIFT_SERVO_LIFTING_POSITION);
                    liftTimer.reset();
                    currentLiftState = LiftState.LIFTING;
                }
                break;
            case LIFTING:
                // After a 500ms delay, return the servo to the down position
                if (liftTimer.milliseconds() > 500) {
                    setLiftPosition(Constants.IntakeConfig.LIFT_SERVO_NOT_LIFTING_POSITION);
                    currentLiftState = LiftState.IDLE;
                }
                break;
        }

        previousCircleButtonState = currentCircleButtonState;
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void setLiftPosition(double position) {
        liftServo.setPosition(position);
    }

    /**
     * Returns the current power of the intake motor. Useful for telemetry.
     * @return The current power of the intake motor.
     */
    public double getMotorPower() {
        return intakeMotor.getPower();
    }
    public double getLiftServoPosition(){
        return liftServo.getPosition();
    }

    public double getLeftDistance(DistanceUnit unit) {
        return leftDistanceSensor.getDistance(unit);
    }

    public double getRightDistance(DistanceUnit unit) {
        return rightDistanceSensor.getDistance(unit);
    }

    public boolean isObjectDetected() {
        return getLeftDistance(DistanceUnit.CM) < Constants.IntakeConfig.INTAKE_DISTANCE_THRESHOLD_CM ||
               getRightDistance(DistanceUnit.CM) < Constants.IntakeConfig.INTAKE_DISTANCE_THRESHOLD_CM;
    }
}
