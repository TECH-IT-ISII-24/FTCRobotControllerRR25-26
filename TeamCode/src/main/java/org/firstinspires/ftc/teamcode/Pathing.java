package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Dictionary;
import java.util.Hashtable;
import java.util.List;


@TeleOp(name="PathingTest", group="FINAL")
public final class Pathing extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
//
        waitForStart();

        if (true) {
            Pose2d beginPose = new Pose2d(+63, -24, Math.PI);
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeToLinearHeading(new Vector2d(0, -24), Math.PI/2)
                       //.strafeToLinearHeading(new Vector2d(0, 24), Math.PI /2 )
//                        .strafeToLinearHeading(new Vector2d(-12, -52), - Math.PI/ 2)
//                        .strafeToLinearHeading(new Vector2d(-24, -24), - 3* Math.PI / 4)
                        //.splineTo(new Vector2d(0, 60), Math.PI)
                        .build());
        }


    }
}


