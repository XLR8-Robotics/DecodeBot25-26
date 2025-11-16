package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Constants;

/**
 * Subsystem for controlling the turret mechanism.
 */
public class Turret {
    
    /**
     * Enum for turret operating states.
     */
    public enum TurretState {
        AIMING,    // Automatic aiming mode using PID control
        MANUAL     // Manual control mode
    }
    
    private final DcMotorEx turretMotor;
    private final Servo shooterBlocker;
    private final DigitalChannel turretLimitLeft;
    private final DigitalChannel turretLimitRight;

    private boolean isShooterBlocked = true;
    private boolean previousSquareButtonState = false;
    
    // Aiming state variables
    private TurretState currentState = TurretState.MANUAL;
    private double targetFieldAngle = 0.0;
    
    // PID control variables
    private double integral = 0;
    private double previousError = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    public Turret(HardwareMap hardwareMap) {
        this.turretMotor = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.TURRET_MOTOR);
        this.shooterBlocker = hardwareMap.get(Servo.class, Constants.HardwareConfig.SHOOTER_BLOCKER);
        this.turretLimitLeft = hardwareMap.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_LEFT);
        this.turretLimitRight = hardwareMap.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_RIGHT);

        // Set the mode to input
        turretLimitLeft.setMode(DigitalChannel.Mode.INPUT);
        turretLimitRight.setMode(DigitalChannel.Mode.INPUT);

        // Reset the encoder and set the motor to run using encoders.
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Start with the shooter blocker in the blocking position
        setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION);
    }

    /**
     * Sets the turret state and configures motor modes accordingly.
     * @param newState The new state to set (AIMING or MANUAL)
     */
    public void setState(TurretState newState) {
        this.currentState = newState;
        
        if (newState == TurretState.AIMING) {
            // Set motor to position control for aiming
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Reset PID variables
            integral = 0;
            previousError = 0;
            pidTimer.reset();
        } else {
            // Set motor to velocity control for manual operation
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    
    /**
     * Gets the current turret state.
     * @return The current TurretState
     */
    public TurretState getState() {
        return currentState;
    }
    
    /**
     * Sets the target field angle for aiming mode.
     * @param angle The target absolute field angle in radians
     */
    public void setTargetFieldAngle(double angle) {
        this.targetFieldAngle = angle;
    }
    
    /**
     * Gets the current target field angle.
     * @return The target field angle in radians
     */
    public double getTargetFieldAngle() {
        return targetFieldAngle;
    }
    
    /**
     * Update method for aiming mode. Call this when in AIMING state.
     * @param currentRobotHeading The current robot heading in degrees
     */
    public void update(double currentRobotHeading) {
        if (currentState != TurretState.AIMING) {
            return;
        }
        
        // i. Calculate the robot-relative target angle
        double currentRobotHeadingRadians = Math.toRadians(currentRobotHeading);
        double robotRelativeAngle = targetFieldAngle - currentRobotHeadingRadians;
        
        // Normalize angle to [-π, π]
        while (robotRelativeAngle > Math.PI) {
            robotRelativeAngle -= 2 * Math.PI;
        }
        while (robotRelativeAngle < -Math.PI) {
            robotRelativeAngle += 2 * Math.PI;
        }
        
        // ii. Convert this angle to motor encoder ticks
        int targetPosition = angleToTicks(robotRelativeAngle);
        
        // iii. Use PID control to drive the motor to the calculated position
        double currentAngle = getAngle() * (Math.PI / 180.0); // Convert to radians
        double error = robotRelativeAngle - currentAngle;
        
        // PID calculations
        double deltaTime = pidTimer.seconds();
        pidTimer.reset();
        
        integral += error * deltaTime;
        double derivative = (deltaTime > 0) ? (error - previousError) / deltaTime : 0;
        
        double output = Constants.TurretAimingConfig.AIMING_KP * error +
                       Constants.TurretAimingConfig.AIMING_KI * integral +
                       Constants.TurretAimingConfig.AIMING_KD * derivative;
        
        // Apply limit switch logic
        if (isLeftLimitPressed() && output < 0) {
            output = 0;
        }
        if (isRightLimitPressed() && output > 0) {
            output = 0;
        }
        
        // Set target position and power
        turretMotor.setTargetPosition(targetPosition);
        turretMotor.setPower(Math.max(-1.0, Math.min(1.0, output)));
        
        previousError = error;
    }
    
    /**
     * Manual update method for gamepad control.
     * Works regardless of turret state for backward compatibility.
     * @param gamepad The gamepad for manual control
     */
    public void manualUpdate(Gamepad gamepad) {
        // Ensure turret is in manual mode for proper operation
        if (currentState != TurretState.MANUAL) {
            setState(TurretState.MANUAL);
        }
        
        double turretPower = 0;
        if (gamepad.left_bumper) {
            turretPower = -Constants.TurretConfig.TURRET_SPEED;
        } else if (gamepad.right_bumper) {
            turretPower = Constants.TurretConfig.TURRET_SPEED;
        }

        // Apply limit switch logic
        if (isLeftLimitPressed() && turretPower < 0) {
            turretPower = 0;
        }
        if (isRightLimitPressed() && turretPower > 0) {
            turretPower = 0;
        }

        setPower(turretPower);

        // Shooter blocker toggle with square button
        boolean currentSquareButtonState = gamepad.square;
        if (currentSquareButtonState && !previousSquareButtonState) {
            isShooterBlocked = !isShooterBlocked; // Toggle the state
            if (isShooterBlocked) {
                setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION);
            } else {
                setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_ZERO_POSITION);
            }
        }
        previousSquareButtonState = currentSquareButtonState;
    }
    
    /**
     * Legacy update method for backward compatibility.
     * Simply calls manualUpdate() to maintain existing functionality.
     * @param gamepad The gamepad for manual control
     */
    public void update(Gamepad gamepad) {
        manualUpdate(gamepad);
    }
    
    /**
     * Converts an angle in radians to motor encoder ticks.
     * @param angleRadians The angle in radians
     * @return The corresponding encoder ticks
     */
    private int angleToTicks(double angleRadians) {
        double angleDegrees = Math.toDegrees(angleRadians);
        double revolutions = angleDegrees / 360.0;
        double motorRevolutions = revolutions * Constants.TurretConfig.TURRET_GEAR_RATIO;
        return (int) (motorRevolutions * Constants.TurretConfig.TURRET_TICKS_PER_REV);
    }

    public void setPower(double power) {
        turretMotor.setPower(power);
    }

    public void setShooterBlockerPosition(double position) {
        shooterBlocker.setPosition(position);
    }

    public double getMotorPower() {
        return turretMotor.getPower();
    }

    public double getShooterBlockerPosition() {
        return shooterBlocker.getPosition();
    }

    /**
     * Calculates the current angle of the turret in degrees.
     * @return The turret's angle.
     */
    public double getAngle() {
        double ticks = turretMotor.getCurrentPosition();
        double revolutions = ticks / Constants.TurretConfig.TURRET_TICKS_PER_REV;
        double turretRotations = revolutions / Constants.TurretConfig.TURRET_GEAR_RATIO;
        return turretRotations * 360;
    }

    public int getEncoderTicks() {
        return turretMotor.getCurrentPosition();
    }

    /**
     * Checks if the left magnetic limit switch is pressed.
     * @return true if the switch is pressed, false otherwise.
     */
    public boolean isLeftLimitPressed() {
        // The getState() method returns true if the switch is not pressed (open)
        // and false if it is pressed (closed by the magnet). We invert it for intuitive use.
        return !turretLimitLeft.getState();
    }

    /**
     * Checks if the right magnetic limit switch is pressed.
     * @return true if the switch is pressed, false otherwise.
     */
    public boolean isRightLimitPressed() {
        return !turretLimitRight.getState();
    }
}
