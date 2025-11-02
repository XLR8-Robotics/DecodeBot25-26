package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.Constants;

/**
 * This class represents the entire robot, holding all of its subsystems.
 */
public class Robot {

    // Declare all subsystems here
    public Drivetrain drivetrain;
    public Turret turret;
    public Intake intake;
    public Shooter shooter;
    public Limelight limelight;

    /**
     * Initializes all the subsystems on the robot.
     * @param hardwareMap The hardware map from the OpMode.
     */
    public void init(HardwareMap hardwareMap) {
        // Initialize all subsystems here using names from the Constants file
        drivetrain = new Drivetrain(hardwareMap);
        turret = new Turret(hardwareMap, Constants.HardwareConfig.TURRET_MOTOR);
        intake = new Intake(hardwareMap, Constants.HardwareConfig.INTAKE_MOTOR);
        shooter = new Shooter(hardwareMap, Constants.HardwareConfig.SHOOTER_MOTOR, Constants.HardwareConfig.HOOD_SERVO);
        limelight = new Limelight(hardwareMap, Constants.HardwareConfig.LIMELIGHT_NAME);

    }

    // You can add an "update" method here to update all subsystems with one call
    // public void update() {
    //     turret.update(); // This would need to be adjusted if update needs parameters
    // }
}
