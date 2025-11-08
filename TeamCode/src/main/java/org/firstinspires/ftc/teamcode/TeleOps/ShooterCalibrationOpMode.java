package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "Shooter Calibration (Final)", group = "Tuning")
public class ShooterCalibrationOpMode extends LinearOpMode {

    private Robot robot;

    private double currentDistanceIn = 60.0; // Starting distance, adjustable before start
    private static final double DISTANCE_INCREMENT = 6.0; // inches

    // Sequence settings
    private static final double MIN_HOOD = 0.25;
    private static final double MAX_HOOD = 0.75;
    private static final double HOOD_INCREMENT = 0.05;
    private static final double MIN_POWER = 0.50;
    private static final double MAX_POWER = 0.90;
    private static final double POWER_INCREMENT = 0.05;
    private static final int FLYWHEEL_SPIN_UP_MS = 1500;
    private static final int FEED_TIME_MS = 500;

    private static class Shot {
        final double power;
        final double hood;
        Shot(double power, double hood) {
            this.power = power;
            this.hood = hood;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot();
        robot.init(hardwareMap);
        // Send all telemetry to both the Driver Station and the FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // --- Pre-Init Loop: Allow driver to set distance ---
        boolean prevRightBumper = false;
        boolean prevLeftBumper = false;
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.right_bumper && !prevRightBumper) {
                currentDistanceIn += DISTANCE_INCREMENT;
            }
            if (gamepad1.left_bumper && !prevLeftBumper) {
                currentDistanceIn = Math.max(0, currentDistanceIn - DISTANCE_INCREMENT);
            }
            prevLeftBumper = gamepad1.left_bumper;
            prevRightBumper = gamepad1.right_bumper;

            telemetry.addLine("--- Shooter Calibration Setup ---");
            telemetry.addData("Set Distance (in)", "%.1f", currentDistanceIn);
            telemetry.addLine("Use Left/Right Bumper to adjust distance.");
            telemetry.addLine("Ready for final step-by-step test.");
            telemetry.update();
        }

        waitForStart();
        if (!opModeIsActive()) return;

        // --- Generate the shot list ---
        List<Shot> shotList = new ArrayList<>();
        for (double hood = MIN_HOOD; hood <= MAX_HOOD + 0.001; hood += HOOD_INCREMENT) {
            for (double power = MIN_POWER; power <= MAX_POWER + 0.001; power += POWER_INCREMENT) {
                shotList.add(new Shot(power, hood));
            }
        }

        telemetry.log().add("\n--- Starting New Calibration Run ---");
        telemetry.log().add(String.format(Locale.US, "Distance: %.1f inches", currentDistanceIn));

        robot.turret.setShooterBlockerPosition(Constants.TurretConfig.SHOOTER_BLOCKER_ZERO_POSITION);

        for (int i = 0; i < shotList.size(); i++) {
            if (!opModeIsActive()) break;

            Shot currentShot = shotList.get(i);
            
            // 1. Display upcoming shot and wait for trigger
            telemetry.clear();
            telemetry.addLine("--- Ready for Next Shot ---");
            telemetry.addData("Shot Number", "%d / %d", i + 1, shotList.size());
            telemetry.addData("Hood", "%.3f", currentShot.hood);
            telemetry.addData("Power", "%.3f", currentShot.power);
            telemetry.addLine("\nLoad a ball.");
            telemetry.addLine("Press (A / Cross) to FIRE this shot.");
            telemetry.update();

            while (opModeIsActive() && !gamepad1.a) { idle(); }
            if (!opModeIsActive()) break;

            // 2. Execute the fire cycle
            telemetry.clear();
            telemetry.addData("Status", "FIRING Shot %d...", i + 1);
            telemetry.update();

            robot.shooter.setHoodPosition(Range.clip(currentShot.hood, 0, 1));
            robot.shooter.setPower(Range.clip(currentShot.power, 0, 1));
            sleep(FLYWHEEL_SPIN_UP_MS);
            robot.intake.setPower(Constants.IntakeConfig.INTAKE_SPEED);
            sleep(FEED_TIME_MS);
            robot.intake.setPower(0);
            robot.shooter.setPower(0);

            // 3. Prompt for result and log it
            telemetry.clear();
            telemetry.addLine("Shot fired. Please log the result.");
            telemetry.addLine("Press (X / Square) for MADE");
            telemetry.addLine("Press (Y / Triangle) for MISSED");
            telemetry.update();
            
            boolean logged = false;
            while(opModeIsActive() && !logged) {
                if(gamepad1.x) {
                    logDataPoint(currentShot.power, currentShot.hood, "MADE");
                    logged = true;
                } else if (gamepad1.y) {
                    logDataPoint(currentShot.power, currentShot.hood, "MISSED");
                    logged = true;
                }
                idle();
            }
            sleep(250); // Debounce
        }

        // --- End of Test ---
        telemetry.log().add("--- Calibration Run Complete ---");
        robot.stopAllMotors();
        telemetry.clear();
        telemetry.log().add("Calibration sequence complete!");
        telemetry.update();
        sleep(10000);
    }

    private void logDataPoint(double power, double hood, String result) {
        double roundedHood = Math.round(hood * 1000.0) / 1000.0;
        double roundedPower = Math.round(power * 1000.0) / 1000.0;

        String dataFormat = String.format(
            Locale.US,
            "{ distance: %.1f, power: %.3f, hood: %.3f, result: \"%s\" }",
            currentDistanceIn,
            roundedPower,
            roundedHood,
            result
        );
        telemetry.log().add(dataFormat);
        telemetry.update();
    }
}
