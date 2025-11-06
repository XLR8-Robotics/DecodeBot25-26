package org.firstinspires.ftc.teamcode.Autos.commands;

import org.firstinspires.ftc.teamcode.TuningConfig;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Drive along +X (forward) or -X by a delta in inches using teleop drive.
 */
public class DriveDeltaX implements AutoCommand {
    private final double deltaInches;
    private final double power;
    private final double toleranceIn;
    private double startX;

    public DriveDeltaX(double deltaInches) {
        this(deltaInches, TuningConfig.AUTO_DRIVE_POWER, TuningConfig.AUTO_DISTANCE_TOLERANCE_IN);
    }

    public DriveDeltaX(double deltaInches, double power, double toleranceIn) {
        this.deltaInches = deltaInches;
        this.power = power;
        this.toleranceIn = toleranceIn;
    }

    @Override
    public void init(Robot robot) {
        startX = robot.getDriveTrain().getPose().getX();
        robot.getDriveTrain().getFollower().startTeleopDrive(true);
    }

    @Override
    public void execute(Robot robot) {
        double sign = Math.signum(deltaInches);
        robot.getDriveTrain().getFollower().setTeleOpDrive(sign * power, 0.0, 0.0, true);
        robot.getDriveTrain().update();
    }

    @Override
    public boolean isFinished(Robot robot) {
        double traveled = robot.getDriveTrain().getPose().getX() - startX;
        return Math.abs(traveled - deltaInches) <= toleranceIn;
    }

    @Override
    public void end(Robot robot) {
        robot.getDriveTrain().getFollower().setTeleOpDrive(0.0, 0.0, 0.0, true);
    }
}


