package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.config.Constants;

import java.util.Arrays;
import java.util.List;

/**
 * Subsystem for controlling the shooter mechanism, including the shooter wheel and hood.
 * Supports both manual and automatic distance-based shooting modes.
 */
public class Shooter {

    private final DcMotorEx shooterMotor;
    private final Servo hoodServo;
    private boolean isRunning;

    private final List<Double> hoodPositions = Arrays.asList(
            Constants.ShooterConfig.HOOD_MIN,
            Constants.ShooterConfig.HOOD_CENTER,
            Constants.ShooterConfig.HOOD_MAX
    );
    private int currentHoodPositionIndex = 2; // Start at center position (index 4)
    private boolean previousDpadUpState = false;
    private boolean previousDpadDownState = false;
    
    // Automatic shooting variables
    private double calculatedRPM = 0.0;        // For RPM-based control
    private double calculatedHoodPosition = 0.0;
    private boolean rpmControlEnabled = Constants.AutoShootingConfig.USE_RPM_CONTROL;

    public Shooter(HardwareMap hardwareMap) {
        this.shooterMotor = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.SHOOTER_MOTOR);
        this.hoodServo = hardwareMap.get(Servo.class, Constants.HardwareConfig.HOOD_SERVO);
        this.isRunning = false;


        // Set initial hood position
        setHoodPosition(hoodPositions.get(currentHoodPositionIndex));
        
        // Configure motor for velocity control if RPM mode is enabled
        if (rpmControlEnabled) {
            setupVelocityControl();
        }

        // If the shooter motor spins in the wrong direction, you can reverse it by uncommenting the next line.
        // this.shooterMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    private void setupVelocityControl() {
        // Reset encoder for velocity measurements
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set velocity PID coefficients from constants
        PIDFCoefficients pidCoeffs = new PIDFCoefficients(
            Constants.AutoShootingConfig.VELOCITY_KP,
            Constants.AutoShootingConfig.VELOCITY_KI,
            Constants.AutoShootingConfig.VELOCITY_KD,
            Constants.AutoShootingConfig.VELOCITY_KF
        );
        
        // Apply PID coefficients (RUN_USING_ENCODER mode for velocity control)
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoeffs);
        
        // Set zero power behavior to float for smoother control
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // =================================================================================
    // RPM CONTROL METHODS
    // =================================================================================

    public void setRPM(double targetRPM) {
        if (rpmControlEnabled) {
            if(targetRPM == 0)
            {
                shooterMotor.setPower(0);
            } else {
                // Convert RPM to ticks per second for setVelocity()
                double ticksPerSecond = rpmToTicksPerSecond(targetRPM);
                shooterMotor.setVelocity(ticksPerSecond);
            }
            isRunning = targetRPM > 0;
        } else {
            // If RPM control is disabled, fall back to power control
            // This is a rough approximation - you may want to create a better mapping
            double approximatePower = Math.min(1.0, targetRPM / 5000.0); // Assume max RPM of 5000
            setPower(approximatePower);
        }
    }

    public double getCurrentRPM() {
        if (rpmControlEnabled) {
            double currentVelocity = shooterMotor.getVelocity(); // ticks per second
            return ticksPerSecondToRPM(currentVelocity);
        } else {
            return -1; // RPM not available in power control mode
        }
    }

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm / 60.0) * Constants.AutoShootingConfig.SHOOTER_MOTOR_TICKS_PER_REV;
    }

    private double ticksPerSecondToRPM(double ticksPerSecond) {
        return (ticksPerSecond * 60.0) / Constants.AutoShootingConfig.SHOOTER_MOTOR_TICKS_PER_REV;
    }

    public boolean isRPMStable(double targetRPM, double tolerance) {
        if (!rpmControlEnabled) {
            return true; // In power mode, assume it's always "stable"
        }
        
        double currentRPM = getCurrentRPM();
        return Math.abs(currentRPM - targetRPM) <= tolerance;
    }

    public boolean isRPMStable(double targetRPM) {
        return isRPMStable(targetRPM, 50.0);
    }
    
    // =================================================================================
    // MANUAL CONTROL METHODS
    // =================================================================================

    public void manualUpdate(Gamepad gamepad) {
        // --- Shooter Motor Control ---
        // NOTE: Idle speed toggle moved to LaunchSequenceController

        boolean currentDpadUpState = gamepad.dpad_down;
        boolean currentDpadDownState = gamepad.dpad_up;

        if (currentDpadUpState && !previousDpadUpState) {
            // Move to the next hood position
            currentHoodPositionIndex = Math.min(hoodPositions.size() - 1, currentHoodPositionIndex + 1);
            setHoodPosition(hoodPositions.get(currentHoodPositionIndex));
        } else if (currentDpadDownState && !previousDpadDownState) {
            // Move to the previous hood position
            currentHoodPositionIndex = Math.max(0, currentHoodPositionIndex - 1);
            setHoodPosition(hoodPositions.get(currentHoodPositionIndex));
        }

        previousDpadUpState = currentDpadUpState;
        previousDpadDownState = currentDpadDownState;
    }
    public void initializeIdleSpeed(Boolean setIdleSpeed){
        if(setIdleSpeed)
        {
            setRPM(Constants.ShooterConfig.SHOOTER_RPM_IDLE);
        }
        if(!setIdleSpeed)
        {
            setRPM(0);
        }

        isRunning = setIdleSpeed;
    }

    public void update(Gamepad gamepad) {
        manualUpdate(gamepad);
    }
    
    // =================================================================================
    // UTILITY AND GETTER METHODS
    // =================================================================================

    public void setPower(double power) {
        shooterMotor.setPower(power);
        isRunning = power > 0;
    }
    
    public boolean isRunning() {
        return isRunning;
    }

    public void setHoodPosition(double position) {
        hoodServo.setPosition(position);
    }

    /**
     * Returns the current power of the shooter motor. Useful for telemetry.
     * @return The current power of the shooter motor.
     */
    public double getMotorPower() {
        return shooterMotor.getPower();
    }

    /**
     * Returns the current position of the hood servo. Useful for telemetry.
     * @return The current position of the hood servo.
     */
    public double getServoPosition() {
        return hoodServo.getPosition();
    }

    public String getDetailedStatusString() {
        StringBuilder status = new StringBuilder();
        status.append(String.format("Mode: %s | ", rpmControlEnabled ? "RPM" : "Power"));
        
        if (rpmControlEnabled) {
            double currentRPM = getCurrentRPM();
            status.append(String.format("RPM: %.0f → %.0f | ", currentRPM, calculatedRPM));
            if (calculatedRPM > 0) {
                status.append(String.format("Stable: %s | ", isRPMStable(calculatedRPM) ? "✅" : "❌"));
            }
        } else {
            status.append(String.format("Power: %.2f | ", getMotorPower()));
        }
        
        status.append(String.format("Hood: %.2f", calculatedHoodPosition));
        return status.toString();
    }
}
