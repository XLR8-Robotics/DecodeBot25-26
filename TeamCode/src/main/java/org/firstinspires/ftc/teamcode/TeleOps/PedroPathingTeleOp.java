package org.firstinspires.ftc.teamcode.TeleOps;

import com.bylazar.field.FieldManager;
import com.bylazar.field.FieldPresetParams;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutoAimingTurret;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Pedro Pathing TeleOp
 * 
 * This OpMode uses Pedro Pathing's Follower class for drivetrain control
 * and outputs robot location to Panels Dashboard.
 * 
 * Controls:
 * - Gamepad 1: Drivetrain (via Follower)
 * - Robot position is displayed on Panels Dashboard
 */
@TeleOp(name = "TeleOpWithAutoAiming", group = "Game")
public class PedroPathingTeleOp extends LinearOpMode {

    private Follower follower;
    private Robot robot;
    private AutoAimingTurret turret;
    private FieldManager panelsField;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));

        turret = new AutoAimingTurret(hardwareMap, follower, telemetry);

        telemetry.addData("Status", "Pedro Pathing TeleOp Initialized");
        telemetry.addData("Note", "Robot position will be displayed on Panels Dashboard");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            follower.update();
        }
        
        waitForStart();

        while (opModeIsActive()) {
            robot.UpdateGamePad1(gamepad1);
            robot.UpdateGamePad2NoLS(gamepad2);
            turret.RunTurret(telemetry);

        }
    }
}

