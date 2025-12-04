package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.config.Constants;

/**
 * Subsystem for controlling the shooter mechanism, including the shooter wheel and hood.
 * Supports both manual and automatic distance-based shooting modes.
 */
public class Shooter {

    public final DcMotorEx shooterMotor;
    private final Servo hoodServo;
    private boolean isRunning;

    public enum ShooterStates {
        NEAR(Constants.ShooterConfig.SHOOTER_RPM_NEAR, Constants.ShooterConfig.HOOD_POSITION_MEDIUM),
        //MIDDLE(Constants.ShooterConfig.SHOOTER_RPM_MEDIUM, Constants.ShooterConfig.HOOD_POSITION_MEDIUM),
        FAR(Constants.ShooterConfig.SHOOTER_RPM_FAR, Constants.ShooterConfig.HOOD_POSITION_FAR);
        public final double rpm;
        public final double hoodPosition;

        ShooterStates(double rpm, double hoodPosition) {
            this.rpm = rpm;
            this.hoodPosition = hoodPosition;
        }
    }

    public enum ShooterAutoStates{
        NEAR(Constants.ShooterConfig.SHOOTER_RPM_NEAR, Constants.ShooterConfig.HOOD_POSITION_MEDIUM),
        MIDDLE(Constants.ShooterConfig.SHOOTER_RPM_MEDIUM, Constants.ShooterConfig.HOOD_POSITION_MEDIUM),
        FAR(Constants.ShooterConfig.SHOOTER_RPM_FAR, Constants.ShooterConfig.HOOD_POSITION_FAR),
        FARAUTO(Constants.ShooterConfig.SHOOTER_RPM_FAR_AUTO, Constants.ShooterConfig.HOOD_POSITION_FAR_AUTO),
        BLUENEAR(Constants.ShooterConfig.SHOOTER_RPM_NEAR_BLUE, Constants.ShooterConfig.HOOD_POSITION_MEDIUM);
        public final double rpm;
        public final double hoodPosition;

        ShooterAutoStates(double rpm, double hoodPosition) {
            this.rpm = rpm;
            this.hoodPosition = hoodPosition;
        }
    }

    public enum ShooterStateNames{
        NEAR,
        MIDDLE,
        FAR
    }
    public String cState;
    private int currentStateIndex = 0;
    private boolean previousIncrementStateInput = false;
    private boolean previousDecrementStateInput = false;

    // NEW: For long press detection
    private final ElapsedTime trianglePressTimer = new ElapsedTime();
    private boolean isTrianglePressed = false;
    private static final double SHUTOFF_HOLD_TIME_SECONDS = 2.0;
    
    // Tracks if the toggle action has been performed during the current button press
    private boolean shutoffActionExecuted = false;
    // Tracks the disabled state of the shooter motor
    private boolean shooterMotorDisabled = false;

    private double calculatedHoodPosition = 0.0;
    private double targetRPM = 0.0;

    public Shooter(HardwareMap hardwareMap) {
        this.shooterMotor = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.SHOOTER_MOTOR);
        this.hoodServo = hardwareMap.get(Servo.class, Constants.HardwareConfig.HOOD_SERVO);
        this.isRunning = false;

        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        applyState(ShooterStates.NEAR);
        cState = ShooterStateNames.NEAR.name();
    }
    
    public void applyState(ShooterStates state) {
        setRPM(state.rpm);
        setHoodPosition(state.hoodPosition);
    }

    public void applyState(ShooterAutoStates state) {
        setRPM(state.rpm);
        setHoodPosition(state.hoodPosition);
    }

    // =================================================================================
    // RPM CONTROL METHODS
    // =================================================================================
    
    public void setRPM(double targetRPM) {
        this.targetRPM = targetRPM;
        double targetVelocityTicksPerSec = rpmToTicksPerSecond(targetRPM);
        shooterMotor.setVelocity(targetVelocityTicksPerSec);
        
        // Consider the shooter running if the target RPM is significant
        this.isRunning = Math.abs(targetRPM) > 10;
    }

    public void setHoodPosition(double position) {
        hoodServo.setPosition(position);
        calculatedHoodPosition = position;
    }

    public double getCurrentRPM() {
        double currentVelocity = shooterMotor.getVelocity(); // ticks per second
        return ticksPerSecondToRPM(currentVelocity);
    }

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * Constants.AutoShootingConfig.SHOOTER_MOTOR_TICKS_PER_REV) / 60.0;
    }

    private double ticksPerSecondToRPM(double ticksPerSecond) {
        return (ticksPerSecond * 60.0) / Constants.AutoShootingConfig.SHOOTER_MOTOR_TICKS_PER_REV;
    }

    public boolean isRPMStable(double targetRPM, double tolerance) {
        double currentRPM = getCurrentRPM();
        return Math.abs(currentRPM - targetRPM) <= tolerance;
    }

    public boolean isRPMStable(double targetRPM) {
        return isRPMStable(targetRPM, 50.0);
    }

    public void setPIDFCoefficients(double p, double i, double d, double f) {
        com.qualcomm.robotcore.hardware.PIDFCoefficients coefficients = new com.qualcomm.robotcore.hardware.PIDFCoefficients(p, i, d, f);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
    }
    public String getShooterState(){
        return cState;
    }
    public com.qualcomm.robotcore.hardware.PIDFCoefficients getPIDFCoefficients() {
        return shooterMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    // =================================================================================
    // MAIN UPDATE LOOP
    // =================================================================================

    /**
     * Updates the shooter subsystem based on gamepad input.
     * Handles state transitions via D-pad.
     * Handles emergency shutoff via Triangle (hold 2s).
     * @param gamepad The gamepad to read input from.
     */
    public void update(Gamepad gamepad) {
        handleStateTransitionInput(gamepad);
        handleShutoffInput(gamepad);
    }

    private void handleStateTransitionInput(Gamepad gamepad) {
        // Logic inverted in original code? keeping it consistent with intent:
        // dpad_down increments state (towards FAR)
        // dpad_up decrements state (towards NEAR)
        boolean incrementStateInput = gamepad.dpad_down; 
        boolean decrementStateInput = gamepad.dpad_up;

        if (incrementStateInput && !previousIncrementStateInput) {
            // Move to the next state (Limit at max index)
            currentStateIndex = Math.min(ShooterStates.values().length - 1, currentStateIndex + 1);
            applyState(ShooterStates.values()[currentStateIndex]);
            cState = ShooterStates.values()[currentStateIndex].name();
        } else if (decrementStateInput && !previousDecrementStateInput) {
            // Move to the previous state (Limit at 0)
            currentStateIndex = Math.max(0, currentStateIndex - 1);
            applyState(ShooterStates.values()[currentStateIndex]);
            cState = ShooterStates.values()[currentStateIndex].name();
        }

        previousIncrementStateInput = incrementStateInput;
        previousDecrementStateInput = decrementStateInput;
    }

    private void handleShutoffInput(Gamepad gamepad) {
        if (gamepad.triangle) {
            if (!isTrianglePressed) {
                // Button just pressed
                isTrianglePressed = true;
                trianglePressTimer.reset();
                shutoffActionExecuted = false;
            } else {
                // Button held
                if (trianglePressTimer.seconds() >= SHUTOFF_HOLD_TIME_SECONDS) {
                     if (!shutoffActionExecuted) {
                         // Toggle the disabled state
                         shooterMotorDisabled = !shooterMotorDisabled;

                         if (shooterMotorDisabled) {
                             // If now disabled, turn off
                             setRPM(0);
                         } else {
                             // If now enabled, turn on (re-apply state)
                             applyState(ShooterStates.values()[currentStateIndex]);
                         }
                         shutoffActionExecuted = true;
                     }
                }
            }
        } else {
            // Button released
            isTrianglePressed = false;
        }
    }

    public void initializeIdleSpeed(boolean setIdleSpeed){
        if(setIdleSpeed) {
            setRPM(Constants.ShooterConfig.SHOOTER_RPM_IDLE);
        } else {
            setRPM(0);
        }
    }
    
    // =================================================================================
    // UTILITY AND GETTER METHODS
    // =================================================================================
    
    public boolean isRunning() {
        return isRunning;
    }

    public boolean isShooterMotorDisabled() {
        return shooterMotorDisabled;
    }

    public double getMotorPower() {
        return shooterMotor.getPower();
    }
    
    public double getServoPosition() {
        return hoodServo.getPosition();
    }
    
    public ShooterStates getCurrentState() {
        return ShooterStates.values()[currentStateIndex];
    }
    
    public String getDetailedStatusString() {
        StringBuilder status = new StringBuilder();
        status.append("Mode: RPM | ");
        status.append(String.format("State: %s | ", getCurrentState().name()));
        
        double currentRPM = getCurrentRPM();
        status.append(String.format("RPM: %.0f → %.0f | ", currentRPM, targetRPM));
        if (Math.abs(targetRPM) > 10) {
             status.append(String.format("Stable: %s | ", isRPMStable(targetRPM) ? "✅" : "❌"));
        } else {
             status.append(String.format("Stopped%s | ", shooterMotorDisabled ? " (DISABLED)" : ""));
        }
        
        status.append(String.format("Hood: %.2f", calculatedHoodPosition));
        return status.toString();
    }
}
