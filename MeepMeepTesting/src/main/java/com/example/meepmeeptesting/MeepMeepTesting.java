package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-20, -12, Math.PI * ((double) 5/6)))
                        .waitSeconds(3)
                        .turnTo(Math.PI * ((double) 5/4))
                        .setTangent(Math.PI)
                        //.strafeToLinearHeading(FixedVector(35, -24) , Math.PI/2)
                        //.strafeTo(FixedVector(57, -24))
                        .strafeToLinearHeading(FixedVector(32 , -24), Math.PI / 2)


                        //.stopAndAdd(setDrive(rampDrive, 1.0))
                        .strafeToLinearHeading(FixedVector(32 , -63), Math.PI / 2, null, new ProfileAccelConstraint(-20, 25))
                        //.stopAndAdd(setDrive(rampDrive, 0))
                        //.stopAndAdd(setDrive(rampDrive, -0.1))
                        .waitSeconds(0.5)
                        //.stopAndAdd(setDrive(rampDrive, 0))
//
//
//                    // move to goal
//                    .setTangent(Math.PI / 2)
                        //.stopAndAdd(setDrive(shooterDrive, shooterPower))
                        .waitSeconds(1)
                        .splineToLinearHeading(FixedPose(-8, -12, (5.0 / 4.0) * -Math.PI), Math.PI)

                        // shoot ball
                        //.stopAndAdd((setDrive(rampDrive, 1.0)))

                        // turn off all
                        //.waitSeconds(5.0)
                        //.stopAndAdd(setDrive(shooterDrive, 0))
                        //.stopAndAdd(setDrive(rampDrive, 0))
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