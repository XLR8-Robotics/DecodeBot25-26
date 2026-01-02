package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.config.Constants;


public class Turret {
    public final DcMotorEx turretMotor;
    private final Servo shooterBlocker;
    private final DigitalChannel turretLimitLeft;
    private final DigitalChannel turretLimitRight;
    private boolean isShooterBlocked = true;
    private boolean previousSquareButtonState = false;

    private static final int TICKS_PER_REV = 4096;
    private static final double GEAR_RATIO = 6.25;


    public PIDFController turretPID;

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


    public void update(){
        double curAngle = getAngle();

        turretPID.updatePosition(curAngle);
        setPower(turretPID.run());
    }

    public double ticksToDegrees(int ticks){
        double motorRevs = ticks / (double) TICKS_PER_REV;
        double turretRevs = motorRevs / GEAR_RATIO;
        return turretRevs * 360.0;
    }

    public  double getAngle(){
        return ticksToDegrees(turretMotor.getCurrentPosition());
    }

    public void manualUpdate(@NonNull Gamepad gamepad) {

        double turretPower = 0;
        if (gamepad.left_bumper && !isLeftLimitPressed())
        {
            turretPower = Constants.TurretConfig.TURRET_SPEED;
        }
        else if (gamepad.right_bumper && !isRightLimitPressed())
        {
            turretPower = -Constants.TurretConfig.TURRET_SPEED;
        } else
        {
                turretPower = 0;
        }

        setPower(turretPower);

        boolean currentSquareButtonState = gamepad.square;
        if (currentSquareButtonState && !previousSquareButtonState)
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

    public void setPower(double power) {

        if (power > 0 && isLeftLimitPressed()) {
            power = 0;
        }

        if (power < 0 && isRightLimitPressed()) {
            power = 0;
        }
        turretMotor.setPower(power);
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
    public double getShooterBlockerPosition() {
        return shooterBlocker.getPosition();
    }
    public boolean isLeftLimitPressed() { return !turretLimitLeft.getState();}
    public boolean isRightLimitPressed() { return !turretLimitRight.getState();}
}
