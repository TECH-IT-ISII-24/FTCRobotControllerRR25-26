package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        int obeliskOffset = 24;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(63, 12, -Math.PI / 2))
                .strafeToLinearHeading(new Vector2d(-12, 12), Math.PI * 0.78)
                .strafeToLinearHeading(new Vector2d(60,12), Math.PI)
                .strafeTo(new Vector2d(60, 48))
                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static Vector2d FixedVector(double x, double y){

        double newx = (x >= 0) ? x - 0 : x + 0;
        double newy = (y >= 0) ? y - 0 : y + 0;
        return new Vector2d(newx, newy);

    }
    public static Pose2d FixedPose(double x, double y, double heading){

        double newx = (x >= 0) ? x - 0 : x + 0;
        double newy = (y >= 0) ? y - 0 : y + 0;
        return new Pose2d(newx, newy, heading);

    }
}