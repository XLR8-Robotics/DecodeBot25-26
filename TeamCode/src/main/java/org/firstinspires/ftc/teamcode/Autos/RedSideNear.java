package org.firstinspires.ftc.teamcode.Autos;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "Red Side Near", group = "Opmode")
public class RedSideNear extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    private Robot robot;
    // Define start and target poses



    private final Pose startPose = new Pose(152, 123, Math.toRadians(45)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(115, 89, Math.toRadians(44)); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
    private final Pose PPGPose = new Pose(134, 84, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose PGPPose = new Pose(166, 84, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose GPPPose = new Pose(134, 64, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose targetPose = new Pose(174, 64, Math.toRadians(0)); // Example target
    private final Pose scorePose2 = new Pose(117, 91, Math.toRadians(28));
    private final Pose scorePose3 = new Pose(117, 91, Math.toRadians(43));
    private final Pose targetPose2 = new Pose(159, 64, Math.toRadians(0)); // Example target
    private PathChain simplePath;
    private Follower follower;
    private TelemetryManager panelsTelemetry;
    private int pathState = 0;

    @Override
    public void runOpMode() {
        // Initialize telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Reset runtime
        runtime.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        robot = new Robot(hardwareMap);
        robot.shooter.applyState(Shooter.ShooterAutoStates.NEAR);
        robot.shooter.setPIDFCoefficients(40 , 0, 2.5, 12);
        while (opModeIsActive()) {
            follower.update();
            panelsTelemetry.update();

            switch (pathState) {
                case 0:
                    delayTime(3000);
                    pathState = 1;
                    break;
                case 1:
                    followPath(follower, buildPath(startPose, scorePose));
                    pathState = 2;
                    break;
                case 2:
                    if(!follower.isBusy())
                    {
                        shootArtifacts();
                        pathState = 3;
                    }
                    break;
                case 3:
                    follower.followPath(buildPath(scorePose, PPGPose));
                    pathState = 4;
                    break;
                case 4:
                    if(!follower.isBusy())
                    {
                        intakeStart();
                        pathState = 5;
                    }
                    break;
                case 5:
                    follower.followPath(buildPath(PPGPose, PGPPose));
                    pathState = 6;
                    break;
                case 6:
                    if(!follower.isBusy())
                    {
                        intakeStop();
                        pathState = 7;
                    }
                    break;
                case 7 :
                    follower.followPath(buildPath(PGPPose, scorePose2));
                    pathState = 8;
                    break;
                case 8:
                    if(!follower.isBusy())
                    {
                        shootArtifacts();
                        pathState = 9;

                    }
                    break;
                case 9:
                    follower.followPath(buildPath(scorePose2, GPPPose));
                    pathState = 10;
                    break;
                case 10:
                    if(!follower.isBusy())
                    {
                        intakeStart();
                        pathState = 11;
                    }
                    break;
                case 11:
                    follower.followPath(buildPath(GPPPose, targetPose));
                    pathState = 12;
                    break;
                case 12:
                    if(!follower.isBusy())
                    {
                        intakeStop();
                        pathState = 13;
                    }
                    break;
                case 13:
                    if(!follower.isBusy()) {
                        follower.followPath(buildPath(targetPose, GPPPose));
                        pathState = 14;
                    }
                    break;
                case 14 :
                    if(!follower.isBusy()) {
                        follower.followPath(buildPath(targetPose, scorePose3));
                        pathState = 15;
                    }
                    break;
                case 15:
                    if(!follower.isBusy())
                    {
                        shootArtifacts();
                        pathState = 16;
                    }
                    break;
                case 16 :
                    follower.followPath(buildPath(scorePose3, targetPose2 ));
                    pathState = -1;
                    break;
            }

            // Optional telemetry
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
    private PathChain buildPath(Pose startPose, Pose targetPose) {
        return follower.pathBuilder()
                .addPath(new BezierLine(startPose, targetPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading())
                .build();
    }
    private PathChain buildPathC(Pose startPose, Pose targetPose) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(startPose, targetPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading())
                .build();
    }
    private void followPath(Follower follower, PathChain path, int nexPos){
        follower.followPath(path);
        pathState = nexPos;
    }
    private void followPath(Follower follower, PathChain path){
        follower.followPath(path);

    }
    private void intakeStart() {
        robot.intake.setPower(0.9);
    }
    private void intakeStop(){
        robot.intake.setPower(0);
    }
    private void shootArtifacts() {
        robot.turret.setShooterUnBlocked();
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        while(timer.milliseconds() <= 2350)
        {
            intakeStart();
        }
        robot.intake.liftUp();
        timer.reset();
        timer.startTime();
        while(timer.milliseconds() <= 350)
        {
           robot.intake.liftUp();
        }
        robot.intake.liftDown();
        robot.turret.setShooterBlocked();
        intakeStop();
    }
    private void delayTime(double delayTime) {
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        String wait;
        while(timer.milliseconds() <= delayTime)
        {
             wait = "wait";
        }
    }
}
