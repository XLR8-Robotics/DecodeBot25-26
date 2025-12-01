package org.firstinspires.ftc.teamcode.TeleOps;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutoAimingTurret;

@TeleOp(name = "Turret PID & Ticks Tuner")
public class TurretTuner extends LinearOpMode {

    @Configurable
    public static class TurretTuningConfig {
        public static double P = 10.0;
        public static double I = 0.0;
        public static double D = 0.0;
        public static double F = 0.0;
        public static double oscillationAmplitude = 20.0;
        public static double oscillationPeriod = 2.0;
        public static double moveDegrees = 45.0;
    }
    private AutoAimingTurret turret;
    private Follower follower;

    private boolean previousCrossState = false;
    private boolean previousTriangleState = false;
    private boolean previousSquareState = false;
    private boolean previousCircleState = false;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        turret = new AutoAimingTurret(hardwareMap, follower);

        telemetry.addLine("Turret Tuner Initialized. Press PLAY.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            Gamepad g = gamepad1;

            // --- Apply PIDF dynamically from Panels ---
            turret.setPIDF(
                    TurretTuningConfig.P,
                    TurretTuningConfig.I,
                    TurretTuningConfig.D,
                    TurretTuningConfig.F
            );

            // --- Debounced Oscillation for PID tuning (PS X button) ---
            if (g.cross && !previousCrossState) {
                turret.oscillateTurretForPID(
                        TurretTuningConfig.oscillationAmplitude,
                        TurretTuningConfig.oscillationPeriod
                );
            }
            previousCrossState = g.cross;

            // --- Debounced Move Fixed Degrees for ticks/degree (PS Triangle button) ---
            if (g.triangle && !previousTriangleState) {
                turret.moveTurretByDegrees(TurretTuningConfig.moveDegrees);
            }
            previousTriangleState = g.triangle;

            // --- Debounced Shooter Blocker Control ---
            if (g.square && !previousSquareState) turret.setShooterBlocked();
            previousSquareState = g.square;

            if (g.circle && !previousCircleState) turret.setShooterUnBlocked();
            previousCircleState = g.circle;

            // --- Telemetry for Panels dashboard ---
            telemetry.addData("Turret Current Angle", "%.2f deg", turret.getCurrentTurretAngle());
            telemetry.addData("Desired Angle", "%.2f deg", turret.getDesiredTurretAngle());
            telemetry.addData("Last Target Field Angle", "%.2f deg", turret.getLastKnownTargetAngleField());
            telemetry.addData("Target Visible", turret.getHasValidTarget());
            telemetry.addData("Turret Status", turret.getTurretStatus());
            telemetry.addData("Left Limit Switch", turret.isLeftLimitPressed());
            telemetry.addData("Right Limit Switch", turret.isRightLimitPressed());
            telemetry.addData("Oscillation Amplitude", TurretTuningConfig.oscillationAmplitude);
            telemetry.addData("Oscillation Period", TurretTuningConfig.oscillationPeriod);
            telemetry.addData("Move Degrees", TurretTuningConfig.moveDegrees);
            telemetry.addData("PIDF", "P=%.2f I=%.2f D=%.2f F=%.2f",
                    TurretTuningConfig.P,
                    TurretTuningConfig.I,
                    TurretTuningConfig.D,
                    TurretTuningConfig.F
            );
            telemetry.update();

            sleep(20); // prevent crash
        }
    }
}
