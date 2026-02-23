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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(63, 24, Math.PI))
                        .setTangent(Math.PI/ 2)
                .strafeToLinearHeading(new Vector2d(35, 24) , Math.PI / 2)
                .strafeToLinearHeading(new Vector2d(35, 63), Math.PI / 2)
                        .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-24, -24, ((double) 5 / 4) * Math.PI), Math.PI)
                //.splineToLinearHeading(new Pose2d(24, 48, -Math.PI/2), Math.PI)
                        //.strafeToLinearHeading(new Vector2d(-35, -36), - Math.PI)
                //.strafeToLinearHeading(new Vector2d(-24, 4), 3 * Math.PI / 4)
                //.splineTo(new Vector2d(0, 60), Math.PI)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}