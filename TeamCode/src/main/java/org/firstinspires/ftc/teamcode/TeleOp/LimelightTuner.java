package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@TeleOp(name = "Limelight Tuner (Panels)", group = "Vision")
public class LimelightTuner extends OpMode {
    private TelemetryManager telemetryM;
    private Limelight limelight;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        limelight = new Limelight();
    }

    @Override
    public void loop() {
        limelight.update();

        telemetryM.debug("Has Targets: " + limelight.hasTargets());
        telemetryM.debug("Best Tag ID: " + limelight.getBestTagId());
        telemetryM.debug("Best Tag Name: " + limelight.getBestTagName());
        telemetryM.debug("tx: " + limelight.getTx());
        telemetryM.debug("ty: " + limelight.getTy());
        double[] pose = limelight.getBotpose();
        telemetryM.debug("botpose[0..2] (m): " + (pose.length > 0 ? pose[0] : Double.NaN) + ", " + (pose.length > 1 ? pose[1] : Double.NaN) + ", " + (pose.length > 2 ? pose[2] : Double.NaN));
        telemetryM.update(telemetry);
    }
}


