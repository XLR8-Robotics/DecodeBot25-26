package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.config.DcMotorConfig;
import org.firstinspires.ftc.teamcode.config.OdometryConfig; // Added import

/**
 * DriveTrain subsystem that wraps Pedro Pathing's Follower.
 *
 * This class centralizes robot motion control for both TeleOp (manual drive)
 * and Autonomous (path following). It is designed to be extended with
 * GoBILDA Pinpoint odometry integration by configuring the Follower's
 * localizer from elsewhere when the device and constants are available.
 */
public class DriveTrain {
    private final Follower follower;
    private final Odometry odometry;
    private Controls controls;

    // Drive motors
    private final DcMotorEx leftFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightFront;
    private final DcMotorEx rightRear;

    public DriveTrain(HardwareMap hardwareMap, String leftFrontName, String leftRearName, String rightFrontName, String rightRearName) {

        this.follower = Constants.createFollower(hardwareMap);
        this.odometry = new Odometry(hardwareMap);

        this.leftFront = hardwareMap.get(DcMotorEx.class, leftFrontName);
        this.leftRear = hardwareMap.get(DcMotorEx.class, leftRearName);
        this.rightFront = hardwareMap.get(DcMotorEx.class, rightFrontName);
        this.rightRear = hardwareMap.get(DcMotorEx.class, rightRearName);

        // Common FTC conventions: reverse right side for mecanum/tank
        this.rightFront.setDirection(DcMotor.Direction.REVERSE);
        this.rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Safer stopping behavior
        this.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public DriveTrain(HardwareMap hardwareMap,
                      DcMotorConfig leftFront, DcMotorConfig leftRear,
                      DcMotorConfig rightFront, DcMotorConfig rightRear) {
        this(hardwareMap,
                leftFront.getHardwareName(),
                leftRear.getHardwareName(),
                rightFront.getHardwareName(),
                rightRear.getHardwareName());
    }

    /**
     * Starts TeleOp driving mode inside Pedro's follower.
     * Use together with drive() calls each loop, followed by update().
     */
    public void startTeleOpDrive() {
        follower.startTeleopDrive(true);
    }

    /**
     * Attach a Controls instance to have DriveTrain read mappings directly each loop.
     */
    public void setControls(Controls controls) {
        this.controls = controls;
    }

    /**
     * Manual drive helper for TeleOp.
     * @param forward forward/backward command (+forward)
     * @param strafe left/right command (+right)
     * @param turn ccw/cw turn command (+ccw)
     */
    public void drive(double forward, double strafe, double turn) {
        follower.setTeleOpDrive(forward, strafe, turn, true);
    }

    /**
     * Sets the follower's starting pose. Useful before autonomous or TeleOp.
     */
    public void setStartingPose(Pose startingPose) {
        follower.setStartingPose(startingPose);
    }

    /**
     * Follow a single Path.
     */
    public void followPath(Path path) {
        follower.followPath(path);
    }

    /**
     * Follow a PathChain.
     */
    public void followPath(PathChain chain) {
        follower.followPath(chain);
    }

    /**
     * @return true if follower is actively executing a path.
     */
    public boolean isBusy() {
        return follower.isBusy();
    }

    /**
     * Must be called every loop to update pathing/localization and output to motors.
     */
    public void update() {
        // Keep odometry fresh even if follower doesn't call into it
        odometry.update();
        follower.update();
    }

    /**
     * Convenience helper for TeleOp: reads Controls and commands follower.
     */
    public void teleop(Gamepad gamepad1, double normalScale, double slowScale) {
        if (controls == null) return;
        boolean precision = controls.get(Controls.Action.DRIVE_PRECISION, gamepad1);
        double scale = precision ? slowScale : normalScale;

        double forward = scale * controls.get(Controls.Analog.DRIVE_FORWARD, gamepad1);
        double strafe = scale * controls.get(Controls.Analog.DRIVE_STRAFE, gamepad1);
        double turn = scale * controls.get(Controls.Analog.DRIVE_TURN, gamepad1);

        drive(forward, strafe, turn);
        update();
    }

    /**
     * Emergency stop for TeleOp drive context.
     */
    public void stop() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0, 0, 0, true);
    }

    /**
     * Access to the underlying follower for advanced usage (builders, tuning, etc.).
     */
    public Follower getFollower() {
        return follower;
    }

    /**
     * Returns current estimated pose from the follower.
     */
    public Pose getPose() {
        return follower.getPose();
    }

    /**
     * Optional: Configure a Pinpoint-based localizer on the follower's pose tracker.
     * Implement wiring where you construct a Pinpoint localizer and set it here.
     * Left intentionally generic to keep compile safety until the exact localizer
     * class and constants are provided.
     */
    public void configurePinpointLocalizer(Object pinpointLocalizer) {
        // Example (pseudo): follower.getPoseTracker().setLocalizer(pinpointLocalizer);
        // Intentionally not referencing external classes to keep this compile-safe.
        // Cast and apply when your Pinpoint localizer class is available in the project.
    }

    /**
     * Configure odometry (GoBILDA Pinpoint) and, if available, install Pedro's Pinpoint localizer into follower.
     */
    public void configureOdometry(OdometryConfig config) { // Signature changed
        odometry.configure(
                config.getDeviceName(),
                config.getXOffsetMm(),
                config.getYOffsetMm(),
                config.isForwardEncoderForward(),
                config.isStrafeEncoderForward(),
                config.isUseFourBarPod()
        );
        Object pedroLoc = odometry.getPedroLocalizer();
        if (pedroLoc != null) {
            try {
                Object poseTracker = follower.getPoseTracker();
                try {
                    poseTracker.getClass().getMethod("setLocalizer", pedroLoc.getClass()).invoke(poseTracker, pedroLoc);
                } catch (NoSuchMethodException e) {
                    // Try with interfaces if direct method not found
                    for (Class<?> iface : pedroLoc.getClass().getInterfaces()) {
                        try {
                            poseTracker.getClass().getMethod("setLocalizer", iface).invoke(poseTracker, pedroLoc);
                            break; // Found and invoked
                        } catch (Exception ignored) {
                            // Continue trying other interfaces
                        }
                    }
                }
            } catch (Exception ignored) {
                // Ignore if localizer cannot be set
            }
        }
    }

    // Basic tank-style power setter (values in [-1, 1])
    public void setMotorPowers(double lf, double lr, double rf, double rr) {
        leftFront.setPower(lf);
        leftRear.setPower(lr);
        rightFront.setPower(rf);
        rightRear.setPower(rr);
    }

    public DcMotorEx getLeftFront() { return leftFront; }
    public DcMotorEx getLeftRear() { return leftRear; }
    public DcMotorEx getRightFront() { return rightFront; }
    public DcMotorEx getRightRear() { return rightRear; }
}
