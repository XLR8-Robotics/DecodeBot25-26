package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;

/**
 * Public helper to use Panels drawing from outside the pedroPathing tuning file.
 * This bridges to the package-private Drawing class defined in Tuning.java.
 */
public class PanelsHelper {
    private static boolean initialized;

    public static void init() {
        if (!initialized) {
            try {
                Drawing.init();
            } catch (Throwable ignored) {}
            initialized = true;
        }
    }

    public static void drawDebug(Follower follower) {
        try {
            Drawing.drawDebug(follower);
        } catch (Throwable ignored) {}
    }
}


