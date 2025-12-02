package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.config.Constants;

@TeleOp(name = "Turret Encoder Calibration")
public class TurretTuner extends LinearOpMode {

    private DcMotorEx turretMotor;

    private boolean prevA = false;
    private boolean prevB = false;

    private boolean startCaptured = false;
    private int startTicks = 0;

    // --- Set the actual rotation you will perform ---
    private static final double ROTATION_DEGREES = 180.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Initialize turret motor ---
        turretMotor = hardwareMap.get(DcMotorEx.class, Constants.HardwareConfig.TURRET_MOTOR);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        turretMotor.setPower(0);

        telemetry.addLine("Turret Encoder Calibration Initialized!");
        telemetry.addLine("Press A to capture start ticks, rotate turret by " + ROTATION_DEGREES + "°, press B to capture end ticks.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;

            // --- Capture start ticks ---
            if (a && !prevA) {
                startTicks = turretMotor.getCurrentPosition();
                startCaptured = true;
                telemetry.addData("Start Ticks", startTicks);
                telemetry.addLine("Now rotate turret by " + ROTATION_DEGREES + "° and press B.");
                telemetry.update();
            }
            prevA = a;

            // --- Capture end ticks and calculate ENCODERTICKS ---
            if (b && !prevB && startCaptured) {
                int endTicks = turretMotor.getCurrentPosition();
                int deltaTicks = endTicks - startTicks;

                // Calculate encoder ticks for a full 360° rotation
                double encoderTicks360 = (deltaTicks / ROTATION_DEGREES) * 360.0;
                double ticksPerDegree = encoderTicks360 / 360.0; // just for clarity

                telemetry.addData("End Ticks", endTicks);
                telemetry.addData("Delta Ticks for " + ROTATION_DEGREES + "°", deltaTicks);
                telemetry.addData("ENCODERTICKS for 360° rotation", encoderTicks360);
                telemetry.addData("TICKS_PER_DEGREE", ticksPerDegree);
                telemetry.addLine("Use ENCODERTICKS in your turret class: " + encoderTicks360);
                telemetry.update();

                startCaptured = false;
            }
            prevB = b;

            sleep(20);
        }
    }
}
