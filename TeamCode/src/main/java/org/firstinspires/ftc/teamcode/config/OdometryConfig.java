package org.firstinspires.ftc.teamcode.config;

/**
 * Enum for odometry/localization device configurations.
 * Start with GoBILDA Pinpoint as an example; extend if needed.
 */
public enum OdometryConfig {
    PINPOINT("pinpoint", -84.0, -168.0, true, true, true);

    private final String deviceName;
    private final double xOffsetMm;
    private final double yOffsetMm;
    private final boolean forwardEncoderForward;
    private final boolean strafeEncoderForward;
    private final boolean useFourBarPod;

    OdometryConfig(String deviceName,
                   double xOffsetMm,
                   double yOffsetMm,
                   boolean forwardEncoderForward,
                   boolean strafeEncoderForward,
                   boolean useFourBarPod) {
        this.deviceName = deviceName;
        this.xOffsetMm = xOffsetMm;
        this.yOffsetMm = yOffsetMm;
        this.forwardEncoderForward = forwardEncoderForward;
        this.strafeEncoderForward = strafeEncoderForward;
        this.useFourBarPod = useFourBarPod;
    }

    public String getDeviceName() { return deviceName; }
    public double getXOffsetMm() { return xOffsetMm; }
    public double getYOffsetMm() { return yOffsetMm; }
    public boolean isForwardEncoderForward() { return forwardEncoderForward; }
    public boolean isStrafeEncoderForward() { return strafeEncoderForward; }
    public boolean isUseFourBarPod() { return useFourBarPod; }
}


