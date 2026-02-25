package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-63, -24, -Math.PI/2))
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(35, -24, -Math.PI/2) , -Math.PI/2)

                        // load ball
                        //.stopAndAdd(setRamp(1.0))
                        .strafeToLinearHeading(new Vector2d(35, -63), -Math.PI / 2)
                        //.stopAndAdd(setRamp(0))

                        // move to goal
                        .setTangent(Math.PI / 2)
                        //.afterTime(0, setShooter(1.0))

                        .splineToLinearHeading(new Pose2d(-24, 24, -(5.0 / 4.0) * Math.PI), Math.PI)

                        // shoot ball
                        //.stopAndAdd(setRamp(1.0))
                        .waitSeconds(2.0)

                        // turn off all
                        //.stopAndAdd(setRamp(0))
                        //.stopAndAdd(setShooter(0))
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}