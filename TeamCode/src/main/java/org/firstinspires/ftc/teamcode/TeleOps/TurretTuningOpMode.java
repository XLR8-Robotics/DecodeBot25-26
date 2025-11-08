package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Config // Make this class's static variables editable from the dashboard
@TeleOp(name = "Turret Tuning", group = "Tuning")
public class TurretTuningOpMode extends LinearOpMode {

    private Turret turret;
    private Limelight limelight;

    // These are the values you will tune from the dashboard
    public static double AIMING_KP = 0.03; // Proportional gain
    public static boolean autoAimEnabled = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        turret = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Ready for Turret Tuning.");
        telemetry.addLine("Connect to http://192.168.43.1:8080/dash");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update limelight data every loop
            limelight.update();

            double turretPower = 0.0;

            if (autoAimEnabled) {
                // --- Automatic Aiming Logic ---
                if (limelight.hasTarget()) {
                    // Get the horizontal error from the limelight
                    double tx = limelight.getTx();

                    // Apply the P-controller formula
                    turretPower = tx * AIMING_KP;
                } else {
                    // No target visible, do nothing
                    turretPower = 0.0;
                }
            } else {
                // --- Manual Turret Control ---
                // Use bumpers for manual override if auto-aim is disabled
                if (gamepad1.right_bumper) {
                    turretPower = 0.4;
                } else if (gamepad1.left_bumper) {
                    turretPower = -0.4;
                }
            }

            // Apply power to the turret, respecting hardware limits
            if (turret.isLeftLimitPressed() && turretPower < 0) {
                turret.setPower(0);
            } else if (turret.isRightLimitPressed() && turretPower > 0) {
                turret.setPower(0);
            } else {
                turret.setPower(turretPower);
            }

            // --- Telemetry ---
            telemetry.addLine("--- Turret Tuning ---");
            telemetry.addData("Auto-Aim Enabled", autoAimEnabled);
            telemetry.addData("Aiming KP", AIMING_KP);
            telemetry.addLine();
            telemetry.addLine("--- Limelight Info ---");
            telemetry.addData("Has Target", limelight.hasTarget());
            telemetry.addData("Target tx (Error)", limelight.getTx());
            telemetry.addLine();
            telemetry.addLine("--- Motor Info ---");
            telemetry.addData("Calculated Turret Power", turretPower);
            telemetry.addData("Actual Turret Power", turret.getMotorPower());
            telemetry.addData("Left Limit Pressed", turret.isLeftLimitPressed());
            telemetry.addData("Right Limit Pressed", turret.isRightLimitPressed());
            telemetry.update();
        }
    }
}
