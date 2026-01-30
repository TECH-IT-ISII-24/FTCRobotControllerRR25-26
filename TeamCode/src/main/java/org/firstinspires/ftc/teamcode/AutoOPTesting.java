package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Dictionary;
import java.util.Hashtable;
import java.util.List;



public final class AutoOPTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        if (true) {
            Pose2d beginPose = new Pose2d(0, 0, 0);
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        }


    }
}