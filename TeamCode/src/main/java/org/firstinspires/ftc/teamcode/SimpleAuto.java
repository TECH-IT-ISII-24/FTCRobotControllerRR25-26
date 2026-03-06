package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="SimpleOP", group="autonomous")
public class SimpleAuto extends LinearOpMode {
    @Override
            public void runOpMode(){
        waitForStart();

        Pose2d beginPose = new Pose2d(63, 12, Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeToLinearHeading(new Vector2d(-12, 12), Math.PI * 0.78)
                        .strafeToLinearHeading(new Vector2d(60,12), Math.PI)
                        .strafeTo(new Vector2d(60, 48))
                        .build()
        );

    }


}

