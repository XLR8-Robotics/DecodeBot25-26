package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Constants;
import java.util.List;

/**
 * Subsystem for controlling the turret mechanism.
 */
public class Turret {
    
    /**
     * Enum for turret operating states.
     */
    public enum TurretState {
        AIMING,    // Automatic aiming mode using PID control
        MANUAL,     // Manual control mode
        SEARCHING // Search mode when target is lost
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

    // --- Auto-Aiming / Searching State ---
    private boolean autoAimEnabled = false;
    private double lastKnownTargetAngleField = 0.0;
    private double lastKnownDistance = 0.0;
    private boolean targetWasVisible = false;
    private final ElapsedTime targetLostTimer = new ElapsedTime();
    private boolean isSearching = false;
    private double searchAngle = Constants.TurretAimingConfig.SEARCH_INITIAL_ANGLE; // Current search sweep angle
    private double searchCenterAngle = 0.0; // Center point of the search
    private boolean searchingRight = true; // Direction of current search sweep
    private final ElapsedTime searchDwellTimer = new ElapsedTime(); // Timer for dwelling at each position
    private boolean isDwelling = false; // Whether we're currently dwelling at a position
    private Limelight limelight;
    private int targetFiducialId = -1;

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
     * Sets the limelight instance for auto-aiming.
     * @param limelight The limelight instance
     */
    public void setLimelight(Limelight limelight) {
        this.limelight = limelight;
    }

    /**
     * Sets the specific fiducial ID to track.
     * @param id The AprilTag ID to track. Set to -1 to track any tag (default).
     */
    public void setTargetFiducialId(int id) {
        this.targetFiducialId = id;
    }

    /**
     * Sets the turret state and configures motor modes accordingly.
     * @param newState The new state to set (AIMING or MANUAL)
     */
    public void setState(TurretState newState) {
        this.currentState = newState;
        
        if (newState == TurretState.AIMING) {
            // Set motor to position control for aiming
            // For AutoAim mode (vision), we use RUN_TO_POSITION with hardware PID
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, 
                new PIDFCoefficients(Constants.TurretAimingConfig.HARDWARE_P, 
                                     Constants.TurretAimingConfig.HARDWARE_I, 
                                     Constants.TurretAimingConfig.HARDWARE_D, 
                                     Constants.TurretAimingConfig.HARDWARE_F));
                                     
            // Reset PID variables for software PID (legacy)
            integral = 0;
            previousError = 0;
            pidTimer.reset();
        } else if (newState == TurretState.SEARCHING) {
             turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else {
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
        // Note: MegaTag 2 updateRobotOrientation is skipped as we don't have continuous IMU
        
        if (autoAimEnabled) {
            updateAutoAim(currentRobotHeading);
        } else if (currentState == TurretState.AIMING) {
            // Legacy software PID aiming logic
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
            if (turretMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                 turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            turretMotor.setPower(Math.max(-1.0, Math.min(1.0, output)));
            
            previousError = error;
        }
    }

    private void updateAutoAim(double robotHeading) {
        if (limelight == null) return;

        double currentTurretAngle = getAngle();
        
        // Find the specific target if ID is set, or any target
        List<LLResultTypes.FiducialResult> results = limelight.getFiducialResults();
        LLResultTypes.FiducialResult target = null;
        
        if (results != null) {
            for (LLResultTypes.FiducialResult res : results) {
                if (targetFiducialId == -1 || res.getFiducialId() == targetFiducialId) {
                    target = res;
                    break;
                }
            }
        }
        
        if (target != null) {
            // TARGET VISIBLE: Track directly using vision
             if (isSearching) {
                turretMotor.setPower(0); 
                isSearching = false;
                searchAngle = Constants.TurretAimingConfig.SEARCH_INITIAL_ANGLE;
                setState(TurretState.AIMING);
            }
            
            double tx = target.getTargetXDegrees();
            
            // Store field centric angle
             lastKnownTargetAngleField = -robotHeading + currentTurretAngle + tx;
             targetWasVisible = true;
             targetLostTimer.reset();
             
             double desiredTurretAngle = currentTurretAngle + tx;
             setTargetAngleInternal(desiredTurretAngle);
             
             // Update distance using this specific target's TY
             double ty = target.getTargetYDegrees();
             double angle = Constants.LimelightConfig.LIMELIGHT_ANGLE + ty;
             lastKnownDistance = (Constants.LimelightConfig.APRIL_TAG_HEIGHT - Constants.LimelightConfig.LIMELIGHT_HEIGHT)
                     / Math.tan(Math.toRadians(angle));

        } else if (targetWasVisible && targetLostTimer.seconds() < Constants.TurretAimingConfig.TARGET_LOST_TIMEOUT && Constants.TurretAimingConfig.IMU_SEARCH_ENABLED) {
             // TARGET LOST: Use Heading-based search (Field Centric Hold)
            double desiredTurretAngle = lastKnownTargetAngleField - (-robotHeading);
             // Normalize angle to [-180, 180]
            while (desiredTurretAngle > 180) desiredTurretAngle -= 360;
            while (desiredTurretAngle < -180) desiredTurretAngle += 360;

            setTargetAngleInternal(desiredTurretAngle);
            isSearching = false;
             setState(TurretState.AIMING);

        } else {
             // TARGET LOST: Tracking disabled or timeout expired
             // Start oscillating search pattern
            targetWasVisible = false;
            setState(TurretState.SEARCHING);

            if (!isSearching) {
                isSearching = true;
                searchCenterAngle = currentTurretAngle;
                searchAngle = Constants.TurretAimingConfig.SEARCH_INITIAL_ANGLE;
                searchingRight = true;
                isDwelling = false;
            }
             // Check if we're dwelling (waiting at a position)
            if (isDwelling) {
                // Wait for dwell time before moving to next position
                if (searchDwellTimer.seconds() >= Constants.TurretAimingConfig.SEARCH_DWELL_TIME) {
                    isDwelling = false; // Done dwelling, move to next position
                } else {
                    // Still dwelling, don't move yet
                    return;
                }
            }
            
            double targetAngle;
            boolean shouldMove = false;

            if (searchingRight) {
                targetAngle = searchCenterAngle + searchAngle;
                 if (currentTurretAngle >= targetAngle - 2) {
                    searchingRight = false;
                    isDwelling = true;
                    searchDwellTimer.reset();
                 } else {
                     shouldMove = true;
                 }
            } else {
                targetAngle = searchCenterAngle - searchAngle;
                if (currentTurretAngle <= targetAngle + 2) {
                     searchingRight = true;
                    isDwelling = true;
                    searchDwellTimer.reset();
                     searchAngle = Math.min(
                        searchAngle + Constants.TurretAimingConfig.SEARCH_ANGLE_INCREMENT,
                        Constants.TurretAimingConfig.SEARCH_MAX_ANGLE
                    );
                } else {
                    shouldMove = true;
                }
            }
            
             if (shouldMove) {
                // Move to target position with reduced power
                setTargetAngleInternal(targetAngle, Constants.TurretAimingConfig.SEARCH_POWER);
            }
        }
    }

    private void setTargetAngleInternal(double angle) {
        setTargetAngleInternal(angle, Constants.TurretConfig.TURRET_SPEED);
    }

    private void setTargetAngleInternal(double angle, double power) {
        int targetPositionTicks = angleToTicks(Math.toRadians(angle));
        turretMotor.setTargetPosition(targetPositionTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(power);
    }

    public void enableAutoAim(boolean enable) {
        this.autoAimEnabled = enable;
        if (!enable) {
            setState(TurretState.MANUAL);
        } else {
             setState(TurretState.AIMING);
        }
    }
    
    public boolean isTracking() {
        return autoAimEnabled && targetWasVisible;
    }
    
    public boolean isSearching() {
        return isSearching;
    }

    public double getLastKnownDistance() {
        return lastKnownDistance;
    }
    
    /**
     * Manual update method for gamepad control.
     * Works regardless of turret state for backward compatibility.
     * @param gamepad The gamepad for manual control
     */
    public void manualUpdate(Gamepad gamepad) {
        // Ensure turret is in manual mode for proper operation
        if (currentState != TurretState.MANUAL && !autoAimEnabled) {
            setState(TurretState.MANUAL);
        }
        
        // Override auto-aim if manual input is detected
        if (Math.abs(gamepad.left_stick_x) > 0.1 || gamepad.left_bumper || gamepad.right_bumper) {
             enableAutoAim(false);
        }

        if (!autoAimEnabled) {
            double turretPower = 0;

            if (gamepad.left_bumper && !isLeftLimitPressed()) {
                turretPower = Constants.TurretConfig.TURRET_SPEED;
            } else if (gamepad.right_bumper && !isRightLimitPressed()) {
                turretPower = -Constants.TurretConfig.TURRET_SPEED;
            } else {
                turretPower = 0;
            }
             setPower(turretPower);
        }

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
