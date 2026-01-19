package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;



/**
 * Main Robot class that manages all subsystems and provides both simple manual control
 * and automated launch sequences. Supports both legacy and enhanced aiming modes.
 */
public class Robot {

    // =================================================================================
    // SUBSYSTEM INSTANCES
    // =================================================================================
    
    // Core subsystems
    public BasicDriveTrain basicDriveTrain;
    public Intake intake;
    public Shooter shooter;
    public Turret turret;
    public Follower follower;
    public AutoAimingTurret autoAimingTurret;


    // =================================================================================
    // ENUMS AND CONFIGURATION
    // =================================================================================
    
    /** Enum for target side selection */
    public enum TargetSide {
        RED,
        BLUE
    }

    private TargetSide currentTargetSide = TargetSide.BLUE;

    private boolean autoAimEnabled = false;
    private boolean previousSelect = false;
    // =================================================================================
    // CONSTRUCTORS
    // =================================================================================
    public Robot(HardwareMap hardwareMap) {
        this.basicDriveTrain = new BasicDriveTrain(hardwareMap);
        this.intake = new Intake(hardwareMap);
        this.shooter = new Shooter(hardwareMap);
        this.turret = new Turret(hardwareMap);
    }
    public Robot(HardwareMap hardwareMap, Follower follower) {
            this.basicDriveTrain = new BasicDriveTrain(hardwareMap);
            this.intake = new Intake(hardwareMap);
            this.shooter = new Shooter(hardwareMap);
            this.follower = follower;
            this.autoAimingTurret = new AutoAimingTurret(hardwareMap, follower);
    }
    public void runAutoAim(Gamepad gamepad) {
        autoAimingTurret.update();
    }
    public void UpdateGamePad1(Gamepad gamepad) {
        updateDriveControl(gamepad);
    }
    public void UpdateGamePad2(Gamepad gamepad) {
        UpdateShootingControls(gamepad);
    }
    public void UpdateGamePad2AutoAim(Gamepad gamepad) {
        UpdateShootingControlsAutoAim(gamepad);
        autoAimingTurret.changeBlockingPose(gamepad);
    }
    public void updateDriveControl(Gamepad gamepad) {

        intake.updateIntake(gamepad);
        basicDriveTrain.drive(gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x);
    }
    public void UpdateShootingControls(Gamepad gamepad) {
        shooter.update(gamepad);
        turret.manualUpdate(gamepad);
        intake.update(gamepad);
    }
    public void UpdateShootingControlsAutoAim(Gamepad gamepad) {
        intake.update(gamepad);
        shooter.update(gamepad);
    }
    public TargetSide getTargetSide() {
        return currentTargetSide;
    }
    public void setTargetSide(TargetSide side) {
        this.currentTargetSide = side;
    }

    public Follower getFollower() { return follower; }
}
