package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class BaseAutoOpMode extends LinearOpMode {

    protected Robot robot;
    protected Follower follower;
    protected ElapsedTime timer = new ElapsedTime();

    // --- Step System ---
    protected interface Step { boolean update(); }

    protected class DelayStep implements Step {
        private final double delayMs;
        DelayStep(double delayMs) { this.delayMs = delayMs; timer.reset(); }
        @Override
        public boolean update() { return timer.milliseconds() >= delayMs; }
    }

    protected class MoveStep implements Step {
        private final PathChain path;
        private boolean started = false;
        MoveStep(Pose start, Pose target) {
            path = follower.pathBuilder()
                    .addPath(new BezierLine(start, target))
                    .setLinearHeadingInterpolation(start.getHeading(), target.getHeading())
                    .build();
        }
        @Override
        public boolean update() {
            if (!started) {
                follower.followPath(path);
                started = true;
            }
            return !follower.isBusy();
        }
    }

    protected class IntakeStep implements Step {
        private final boolean on;
        IntakeStep(boolean on) { this.on = on; }
        @Override
        public boolean update() {
            robot.intake.setPower(on ? 0.9 : 0);
            return true;
        }
    }

    protected class ShootStep implements Step {
        private final double shootTimeMs;
        private boolean started = false;
        ShootStep(double shootTimeMs) { this.shootTimeMs = shootTimeMs; }
        @Override
        public boolean update() {
            if (!started) {
                robot.turret.setShooterUnBlocked();
                robot.intake.setPower(0.9);
                timer.reset();
                started = true;
            }
            if (timer.milliseconds() >= shootTimeMs) {
                robot.intake.liftDown();
                robot.turret.setShooterBlocked();
                robot.intake.setPower(0);
                return true;
            }
            return false;
        }
    }

    // --- Helper to build a path ---
    protected PathChain buildPath(Pose startPose, Pose targetPose) {
        return follower.pathBuilder()
                .addPath(new BezierLine(startPose, targetPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading())
                .build();
    }
}
