package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.Constants;

/**
 * This class represents the entire robot, holding all of its subsystems.
 */
public class Robot {

    // Declare all subsystems here
    public DriveTrain drivetrain;
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
        drivetrain = new DriveTrain(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

    }
}
