package org.firstinspires.ftc.teamcode.Autos;

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

@Autonomous(name = "Red Side Far", group = "Opmode")
public class RedSideFar extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    private Robot robot;
    // Define start and target poses



    private final Pose startPose = new Pose(115, -24, Math.toRadians(90)); // Start Pose of our robot.
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
        Paths paths = new Paths(follower);

        // Reset runtime
        runtime.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        robot = new Robot(hardwareMap, follower);
        robot.shooter.applyState(Shooter.ShooterAutoStates.REDFAR);
        robot.shooter.setPIDFCoefficients(40 , 0, 2.5, 12);
        while (opModeIsActive()) {
            follower.update();
            panelsTelemetry.update();

            switch (pathState) {
                case 0:
                    setTurretAngle(0);
                    delayTime(3000);
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
                    intakeStart();
                    pathState = 4;
                    break;
                case 4:
                    if(!follower.isBusy()) {
                        follower.followPath(paths.Path3, .8, true);
                        pathState = 5;
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
                    pathState = -1;
                    //intakeStart();
                    break;
                case 9:
                    if(!follower.isBusy()) {
                        follower.followPath(paths.Path6);
                        pathState = 10;

                    }
                    break;
                case 10:
                    if(!follower.isBusy()) {
                        delayTime(2000);
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
    private void setTurretAngle(double deg){
        robot.autoAimingTurret.setTargetPosition(deg);
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;


        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(115, -24), new Pose(110, -10))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(68))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(110, -10), new Pose(143, 12))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(143, 12), new Pose(170, 12))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(170, 12), new Pose(110, -10))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(48))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(110, -10), new Pose(130, -7))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(48))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(30.963, 38.198), new Pose(30.742, 3.963)))
                    .setConstantHeadingInterpolation(Math.toRadians(245))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(30.742, 3.963), new Pose(60.244, 8.687)))
                    .setConstantHeadingInterpolation(Math.toRadians(119))
                    .build();
            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60.244, 8.687), new Pose(38.931, 12.166))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();
        }
    }
}
