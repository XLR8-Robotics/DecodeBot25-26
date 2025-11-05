package com.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.meepmeep.config.DriveConstants;

/**
 * MeepMeep visualization class for RoadRunner path planning.
 * 
 * Run this as a standalone Java application to visualize trajectories.
 * In Android Studio: Right-click -> Run 'MeepMeepTesting.main()'
 * 
 * Make sure to adjust the constraints in DriveConstants to match your robot's capabilities.
 */
public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Create a RoadRunner bot entity with constraints from DriveConstants
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(DriveConstants.MAX_VEL, DriveConstants.MAX_ACCEL, 
                               DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL, 
                               DriveConstants.TRACK_WIDTH)
                .build();

        // Define a trajectory sequence using RoadRunner actions
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                // Example: Drive to x=30 inches
                .lineToX(30)
                // Turn 90 degrees
                .turn(Math.toRadians(90))
                // Drive to y=30 inches
                .lineToY(30)
                // Turn another 90 degrees
                .turn(Math.toRadians(90))
                // Return to x=0
                .lineToX(0)
                // Turn 90 degrees
                .turn(Math.toRadians(90))
                // Return to y=0
                .lineToY(0)
                // Final turn to return to original heading
                .turn(Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

