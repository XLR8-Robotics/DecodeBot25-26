package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Config // Make this class's static variables editable from the dashboard
@TeleOp(name = "Turret Tuning", group = "Tuning")
public class TurretTuningOpMode extends LinearOpMode {

    private Turret turret;
    private Limelight limelight;

    // PID controller variables
    private double integral = 0;
    private double previousError = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    // These are the values you will tune from the dashboard
    public static double AIMING_KP = 0.03; // Proportional gain
    public static double AIMING_KI = 0.0;  // Integral gain
    public static double AIMING_KD = 0.0;  // Derivative gain
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
                    double error = limelight.getTx();
                    double dt = pidTimer.seconds();
                    pidTimer.reset();

                    // Integral term
                    integral += error * dt;

                    // Derivative term
                    double derivative = (error - previousError) / dt;

                    // PID formula
                    turretPower = (AIMING_KP * error) + (AIMING_KI * integral) + (AIMING_KD * derivative);

                    previousError = error;
                } else {
                    // No target visible, do nothing and reset PID
                    turretPower = 0.0;
                    integral = 0;
                    previousError = 0;
                }
            } else {
                // --- Manual Turret Control ---
                // Use bumpers for manual override if auto-aim is disabled
                if (gamepad1.right_bumper) {
                    turretPower = 0.4;
                } else if (gamepad1.left_bumper) {
                    turretPower = -0.4;
                }
                 // Reset PID when not in auto-aim mode
                integral = 0;
                previousError = 0;
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
            telemetry.addData("Aiming KI", AIMING_KI);
            telemetry.addData("Aiming KD", AIMING_KD);
            telemetry.addLine();
            telemetry.addLine("--- Limelight Info ---");
            telemetry.addData("Has Target", limelight.hasTarget());
            telemetry.addData("Target tx (Error)", limelight.getTx());
            telemetry.addLine();
            telemetry.addLine("--- Motor Info ---");
            telemetry.addData("Turret Angle", "%.2f degrees", turret.getAngle());
            telemetry.addData("Calculated Turret Power", turretPower);
            telemetry.addData("Actual Turret Power", turret.getMotorPower());
            telemetry.addData("Left Limit Pressed", turret.isLeftLimitPressed());
            telemetry.addData("Right Limit Pressed", turret.isRightLimitPressed());
            telemetry.update();
        }
    }
}
