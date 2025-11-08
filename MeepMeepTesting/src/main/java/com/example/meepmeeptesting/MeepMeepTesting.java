package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    // Set the offset values
    private static double offsetX;
    private static double offsetY;
    private static double offsetHeading;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setStartPose(new Pose2d(-25, 62, Math.toRadians(270))
                )
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-10, 72, Math.toRadians(270)))

                .strafeToLinearHeading(new Vector2d(-5, 30),  Math.toRadians(270),
                        new TranslationalVelConstraint(100.0),
                        new ProfileAccelConstraint(-120, 100))

                .strafeToSplineHeading(new Vector2d(-40, 42),  Math.toRadians(240),
                        new TranslationalVelConstraint(100.0),
                        new ProfileAccelConstraint(-120, 100))

                .turnTo(Math.toRadians(120))
                .turnTo(Math.toRadians(220))
                .turnTo(Math.toRadians(120))
                .turnTo(Math.toRadians(200))

                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}