package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@Config // Make this class's static variables editable from the dashboard
@TeleOp(name = "Limelight Tuning", group = "Tuning")
public class LimelightTuningOpMode extends LinearOpMode {

    private Limelight limelight;

    // These are the values you will tune from the dashboard.
    // Set them to your best-guess initial values.
    public static double LIMELIGHT_HEIGHT = 3.9599;   // inches
    public static double LIMELIGHT_ANGLE = 16.5;    // degrees
    public static double APRIL_TAG_HEIGHT = 30.0;    // inches

    // A known, measured distance to calibrate against.
    public static double ACTUAL_DISTANCE_IN = 72.0; // inches

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        limelight = new Limelight(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Ready for Limelight Distance Tuning.");
        telemetry.addLine("Connect to http://192.168.43.1:8080/dash");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update limelight data every loop
            limelight.update();

            double calculatedDistance = 0.0;

            if (limelight.hasTarget()) {
                double ty = limelight.getTy(); // Vertical offset in degrees
                double angle = LIMELIGHT_ANGLE + ty;
                calculatedDistance = (APRIL_TAG_HEIGHT - LIMELIGHT_HEIGHT)
                        / Math.tan(Math.toRadians(angle));
            } else {
                calculatedDistance = 0.0;
            }

            // --- Telemetry ---
            telemetry.addLine("--- Limelight Distance Tuning ---");
            telemetry.addLine("Place the robot at the ACTUAL_DISTANCE from the target.");
            telemetry.addLine("Adjust constants until Calculated Distance matches Actual Distance.");
            telemetry.addLine();
            telemetry.addData("Limelight Height", LIMELIGHT_HEIGHT);
            telemetry.addData("Limelight Angle", LIMELIGHT_ANGLE);
            telemetry.addData("AprilTag Height", APRIL_TAG_HEIGHT);
            telemetry.addLine();
            telemetry.addData("Actual Measured Distance", ACTUAL_DISTANCE_IN);
            telemetry.addData("Calculated Distance", calculatedDistance);
            telemetry.addData("Error", ACTUAL_DISTANCE_IN - calculatedDistance);
            telemetry.addLine();
            telemetry.addData("Limelight Has Target", limelight.hasTarget());
            telemetry.addData("Limelight TY (Vertical Offset)", limelight.getTy());

            telemetry.update();
        }
    }
}
