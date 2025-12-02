package org.firstinspires.ftc.teamcode.Autos;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "Red Side Near V2", group = "Opmode")
public class RedSideNearV2 extends BaseAutoOpMode {

    private TelemetryManager panelsTelemetry;
    private int currentStepIndex = 0;
    private Step[] steps;

    // Start and target poses
    private final Pose startPose = new Pose(152, 123, Math.toRadians(45));
    private final Pose scorePose = new Pose(115, 89, Math.toRadians(44));
    private final Pose PPGPose = new Pose(134, 84, Math.toRadians(0));
    private final Pose PGPPose = new Pose(164, 84, Math.toRadians(0));
    private final Pose scorePose2 = new Pose(117, 91, Math.toRadians(30));

    @Override
    public void runOpMode() {
        // Initialize telemetry and follower
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize robot
        robot = new org.firstinspires.ftc.teamcode.subsystems.Robot(hardwareMap);
        robot.shooter.applyState(Shooter.ShooterAutoStates.NEAR);
        robot.shooter.setPIDFCoefficients(40, 0, 2.5, 12);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // --- Define autonomous sequence ---
        steps = new Step[] {
                new DelayStep(3000),
                new MoveStep(startPose, scorePose),
                new ShootStep(2350),
                new MoveStep(scorePose, PPGPose),
                new IntakeStep(true),
                new MoveStep(PPGPose, PGPPose),
                new IntakeStep(false),
                new MoveStep(PGPPose, scorePose2),
                new ShootStep(2350)
        };

        waitForStart();
        timer.reset();

        // --- Main loop ---
        while (opModeIsActive() && currentStepIndex < steps.length) {
            Step currentStep = steps[currentStepIndex];
            if (currentStep.update()) {
                currentStepIndex++;
                timer.reset();
            }

            // Update follower and telemetry
            follower.update();
            panelsTelemetry.update();

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Step", currentStepIndex);
            telemetry.update();
        }
    }
}
