package org.firstinspires.ftc.teamcode.Autos;

import android.graphics.Path;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "Blue Side Far", group = "Opmode")
public class BlueSideFar extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    private Robot robot;
    // Define start and target poses



    private final Pose startPose = new Pose(55.299, 7.52, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(110, -10, Math.toRadians(73)); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
    private final Pose PPGPose = new Pose(143, 8, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose PGPPose = new Pose(100, 59.5, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose GPPPose = new Pose(100, 35.5, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose targetPose = new Pose(72, 20, Math.toRadians(90)); // Example target


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

        robot = new Robot(hardwareMap, follower);
        robot.shooter.applyState(Shooter.ShooterAutoStates.FARAUTO);
        robot.shooter.setPIDFCoefficients(40 , 0, 2.5, 12);
        Paths paths = new Paths(follower);

        while (opModeIsActive()) {
            follower.update();
            panelsTelemetry.update();

            switch (pathState) {
                case 0:
                    setTurretAngle(0);
                    delayTime(4000);
                    pathState = 1;
                case 1:
                    follower.followPath(paths.Path1);
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
                    follower.followPath(paths.Path2);
                    pathState = 4;
                    break;
                case 4:
                    if(!follower.isBusy()) {
                        follower.followPath(paths.Path3);
                        pathState = 5;
                        intakeStart();
                    }
                    break;
                case 5:
                    if(!follower.isBusy()) {
                        intakeStop();
                        pathState = 6;
                    }
                    break;
                case 6:
                    if(!follower.isBusy()) {
                        follower.followPath(paths.Path4);
                        pathState = 7;

                    }
                    break;
                case 7:
                    if(!follower.isBusy())
                    {
                        shootArtifacts();
                        pathState = 8;
                    }
                    break;
                case 8:

                    follower.followPath(paths.Path5);
                    //intakeStart();
                    pathState = 82 ;
                    break;
                case 82:
                    if(!follower.isBusy()) {
                        robot.shooter.setRPM(0);
                        pathState = -1;
                    }

                    break;
                case 9:
                    if(!follower.isBusy()) {
                        //follower.setMaxPower(.6);
                        follower.followPath(paths.Path6);
                        pathState = 92;

                    }
                    break;
                case 92:
                    if(!follower.isBusy()) {
                        //follower.setMaxPower(.6);
                        follower.followPath(paths.Path62);
                        pathState = 93;
                    //add delay between these later
                    }
                    break;
                case 93:
                    if(!follower.isBusy()) {
                        //follower.setMaxPower(.6);
                        follower.followPath(paths.Path63);
                        pathState = 10;

                    }
                    break;

                case 10:
                    if(!follower.isBusy()) {
                        //follower.setMaxPower(1);
                        intakeStop();
                        pathState = 11;
                    }
                    break;
                case 11:
                    if(!follower.isBusy()) {
                        follower.followPath(paths.Path7);
                        pathState = 12;

                    }
                    break;
                case 12:
                    if(!follower.isBusy())
                    {
                        shootArtifacts();
                        pathState = 13;
                    }
                    break;
                case 13:
                    if(!follower.isBusy()) {
                        follower.followPath(paths.Path8);
                        pathState = -1;

                    }
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
        robot.autoAimingTurret.setShooterUnBlocked();
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
        robot.autoAimingTurret.setShooterBlocked();
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

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path52;
        public PathChain Path6;
        public PathChain Path62;
        public PathChain Path63;
        public PathChain Path7;
        public PathChain Path8;


        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.300, 7.521), new Pose(62.244, 10.687))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(118.5))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(62.244, 10.687), new Pose(52.470, 22.276))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(52.470, 22.276), new Pose(19.378, 22.276))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(19.378, 22.276), new Pose(60.470, 14.276))
                    )
                    //previouisly 125 on angle
                    .setConstantHeadingInterpolation(Math.toRadians(130))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60.470, 14.276), new Pose(38.931, 12.166))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(210))
                    .build();
            Path52 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(19, 5.276), new Pose(40.724, -3))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(210))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(40.724, -3), new Pose(30.742, -3)))
                    .setConstantHeadingInterpolation(Math.toRadians(210))
                    .build();
            Path62 = follower
                    .pathBuilder()
                    //increase x
                    .addPath(
                            new BezierLine(new Pose(30.742, -3), new Pose(40.724, -3))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
            Path63 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.724, -3), new Pose(29.724, -3))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(29.724, -3), new Pose(60.244, 4.687)))
                    .setConstantHeadingInterpolation(Math.toRadians(120))
                    .build();
            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60.244, 4.687), new Pose(38.931, 12.166))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();
        }
    }
    private void setTurretAngle(double deg){
        robot.autoAimingTurret.setTargetPosition(deg);
    }

}
