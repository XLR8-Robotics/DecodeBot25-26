package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Odometry subsystem encapsulating the GoBILDA Pinpoint device.
 * Provides configuration, update, and pose accessors.
 * Optionally supports exposing a Pedro Pinpoint localizer instance via reflection.
 */
public class Odometry {
    private final HardwareMap hardwareMap;
    private GoBildaPinpointDriver pinpoint;
    private Object pedroLocalizer; // optional, if Pedro class is available

    public Odometry(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    /**
     * Configure the Pinpoint device and, if possible, create a Pedro Pinpoint localizer instance.
     */
    public void configure(String deviceName,
                          double xPodOffsetMm,
                          double yPodOffsetMm,
                          boolean forwardEncoderForward,
                          boolean strafeEncoderForward,
                          boolean useFourBarPod) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, deviceName);
        pinpoint.setOffsets(xPodOffsetMm, yPodOffsetMm, DistanceUnit.MM);
        pinpoint.setEncoderResolution(
                useFourBarPod
                        ? GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
                        : GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
        );
        pinpoint.setEncoderDirections(
                forwardEncoderForward ? GoBildaPinpointDriver.EncoderDirection.FORWARD : getReverseEnum(),
                strafeEncoderForward ? GoBildaPinpointDriver.EncoderDirection.FORWARD : getReverseEnum()
        );
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));

        tryCreatePedroLocalizer();
    }

    public void update() {
        if (pinpoint != null) {
            pinpoint.update();
        }
    }

    public Pose2D getPose2D() {
        if (pinpoint == null) return new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);
        return pinpoint.getPosition();
    }

    public GoBildaPinpointDriver getDevice() {
        return pinpoint;
    }

    /**
     * If Pedro's Pinpoint localizer exists on classpath, returns the instance; otherwise null.
     */
    public Object getPedroLocalizer() {
        return pedroLocalizer;
    }

    private void tryCreatePedroLocalizer() {
        pedroLocalizer = null;
        if (pinpoint == null) return;
        try {
            String[] candidateClassNames = new String[]{
                    "com.pedropathing.localization.PinpointLocalizer",
                    "com.pedropathing.localizers.PinpointLocalizer"
            };
            for (String className : candidateClassNames) {
                try {
                    Class<?> clazz = Class.forName(className);
                    try {
                        pedroLocalizer = clazz.getConstructor(GoBildaPinpointDriver.class).newInstance(pinpoint);
                        break;
                    } catch (NoSuchMethodException ignored) {}
                    try {
                        pedroLocalizer = clazz.getConstructor(HardwareMap.class, GoBildaPinpointDriver.class).newInstance(hardwareMap, pinpoint);
                        break;
                    } catch (NoSuchMethodException ignored) {}
                } catch (ClassNotFoundException ignored) {}
            }
        } catch (Exception ignored) {}
    }

    // Handle SDK differences: some versions use REVERSE, others REVERSED
    private GoBildaPinpointDriver.EncoderDirection getReverseEnum() {
        try {
            return GoBildaPinpointDriver.EncoderDirection.valueOf("REVERSE");
        } catch (IllegalArgumentException e) {
            try {
                return GoBildaPinpointDriver.EncoderDirection.valueOf("REVERSED");
            } catch (IllegalArgumentException e2) {
                return GoBildaPinpointDriver.EncoderDirection.FORWARD; // safe fallback
            }
        }
    }
}


