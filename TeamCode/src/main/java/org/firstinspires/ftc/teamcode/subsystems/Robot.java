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
        drivetrain = new DriveTrain(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        limelight = new Limelight(hardwareMap);

    }

    // You can add an "update" method here to update all subsystems with one call
    // public void update() {
    //     turret.update(); // This would need to be adjusted if update needs parameters
    // }
}
