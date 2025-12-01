package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.config.Constants;


public class AutoAimingTurret {

    // --- Hardware ---
    private DcMotorEx turretMotor;
    private Limelight3A limelight;
    private Follower follower;
    private final DigitalChannel turretLimitLeft;
    private final DigitalChannel turretLimitRight;
    private final Servo shooterBlocker;

    // TODO: Tune this value for your specific turret motor and gear ratio.
    // This is for a GoBILDA 5203 series motor (537.7 Ticks/Rev) with a 1:1 gear ratio.
    private static final double TICKS_PER_DEGREE = 537.7 / 360.0;

    // PIDF Coefficients for the turret motor's RUN_TO_POSITION mode.
    // TODO: Tune these coefficients for your turret for snappy and accurate movements.
    private static final double P = 10.0; // Proportional - start around 10
    private static final double I = 0.0; // Integral - keep 0 for now
    private static final double D = 0.0; // Derivative - keep 0 for now
    private static final double F = 0.0; // Feedforward - can be 0
    private static final double MAX_POWER = 0.8;
    private static final double TARGET_LOST_TIMEOUT = 2.0; // seconds

    // --- State Variables ---
    private double lastKnownTargetAngleField = 0.0;
    private boolean targetWasVisible = false;
    private final ElapsedTime targetLostTimer = new ElapsedTime();
    private boolean isShooterBlocked = true;
    private boolean previousSquareButtonState = false;

    public enum TurrentStatus{
        INITIALIZED("Initialized"),
        TRACKING("Tracking"),
        SEARCHING("Searching");
        private final String description;
        TurrentStatus(String description) {
            this.description = description;
        }
        public String getDescription() {
            return description;
        }
    };
    private double robotHeading;
    private double currentTurretAngle;
    private String hasValidTarget;
    private double desiredTurretAngle;
    private String turretStatus;

    public AutoAimingTurret(HardwareMap hardwareMap, Follower follower) {
        try {
            this.follower = follower;

            // --- Turret Motor Initialization ---
            turretMotor = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.TURRET_MOTOR);
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Start in velocity control

            // Set the PIDF coefficients for position control
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
            turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);

            shooterBlocker = hardwareMap.get(Servo.class, Constants.HardwareConfig.SHOOTER_BLOCKER);
            turretLimitLeft = hardwareMap.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_LEFT);
            turretLimitRight = hardwareMap.get(DigitalChannel.class, Constants.HardwareConfig.TURRET_LIMIT_RIGHT);

            // Set the mode to input
            turretLimitLeft.setMode(DigitalChannel.Mode.INPUT);
            turretLimitRight.setMode(DigitalChannel.Mode.INPUT);

            // --- Limelight Initialization ---
            limelight = hardwareMap.get(Limelight3A.class, Constants.HardwareConfig.LIMELIGHT_NAME);
            limelight.pipelineSwitch(0); // Use your AprilTag pipeline
            limelight.start();

            targetLostTimer.reset();
            turretStatus = TurrentStatus.INITIALIZED.description;

        } catch (Exception e) {
            throw new RuntimeException("Turret initialization failed: " + e.getMessage());
        }
    }
    public void RunTurret() {
        try {
            // --- Get Current State ---
            double robotHeading = getRobotHeading();
            double currentTurretAngle = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;

            // --- Vision Processing ---
            LLResult result = limelight.getLatestResult();

            hasValidTarget = hasValidTarget(result) ? "Yes" : "No";

            if (hasValidTarget(result)) {

                // TARGET IS VISIBLE: Recalculate the field-centric angle.
                double tx = result.getFiducialResults().get(0).getTargetXDegrees();

                // Field Angle = Robot's Angle + Turret's Angle relative to Robot + Target's Angle relative to Turret
                lastKnownTargetAngleField = robotHeading + currentTurretAngle + tx;

                targetWasVisible = true;
                targetLostTimer.reset(); // Reset the timer since we found the target
            }

            // --- Control Logic ---
            if (targetWasVisible && targetLostTimer.seconds() < TARGET_LOST_TIMEOUT) {
                // AIM AT TARGET: We have a target to aim at (either visible now or recently).
                // Calculate the desired turret angle to counteract robot rotation.
                desiredTurretAngle = lastKnownTargetAngleField - robotHeading;

                // Convert the desired angle to motor encoder ticks.
                int targetPositionTicks = (int) (desiredTurretAngle * TICKS_PER_DEGREE);

                // Command the motor to move to the target position.
                turretMotor.setTargetPosition(targetPositionTicks);
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(MAX_POWER); // Apply power to move to the position

                turretStatus = TurrentStatus.TRACKING.description;

            } else {
                // TARGET LOST: Stop the turret, it will hold its last position due to BRAKE behavior.
                turretMotor.setPower(0);
                targetWasVisible = false; // Give up until we see the target again
                turretStatus = TurrentStatus.SEARCHING.description;
            }
        } catch (Exception e) {
            turretMotor.setPower(0);
        }
    }
    public double getRobotHeading() {
        return Math.toDegrees(follower.getHeading());
    }
    public double getCurrentRobotHeading() { return robotHeading; }
    public double getCurrentTurretAngle(){ return currentTurretAngle; }
    public double getDesiredTurretAngle() { return desiredTurretAngle; }
    public double getLastKnownTargetAngleField() { return lastKnownTargetAngleField; }
    public double getTargetLostTimer() { return targetLostTimer.seconds(); }
    public String getHasValidTarget() {return hasValidTarget;}
    public String getTurretStatus() { return turretStatus; }
    private boolean hasValidTarget(LLResult result) {
        if (result == null || !result.isValid()) {
            return false;
        }
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        return fiducials != null && !fiducials.isEmpty();
    }
    public boolean isLeftLimitPressed() { return !turretLimitLeft.getState();}
    public boolean isRightLimitPressed() { return !turretLimitRight.getState();}
    public void manualUpdate(Gamepad gamepad) {

        boolean currentSquareButtonState = gamepad.square;
        if (currentSquareButtonState)
        {
            isShooterBlocked = !isShooterBlocked; // Toggle the state
            if (isShooterBlocked)
            {
                setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION);
            } else
            {
                setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_ZERO_POSITION);
            }
        }
        previousSquareButtonState = currentSquareButtonState;
    }
    public void setShooterBlocked(){
        setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_BLOCKING_POSITION);
    }
    public void setShooterUnBlocked(){
        setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_ZERO_POSITION);
    }
    public void setShooterBlockerPosition(double position) {
        shooterBlocker.setPosition(position);
    }

    public void stop() {
        try {
            if (turretMotor != null) {
                turretMotor.setPower(0);
            }
            if (limelight != null) {
                limelight.stop();
            }
        } catch (Exception e) {
            // Fail silently on stop
        }
    }
}
