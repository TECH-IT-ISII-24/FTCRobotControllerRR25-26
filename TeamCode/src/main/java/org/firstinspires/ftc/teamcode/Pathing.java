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



public final class Pathing extends LinearOpMode {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    VisionPortal.Builder builder = new VisionPortal.Builder();





    @Override
    public void runOpMode() throws InterruptedException {
//        builder.setCamera(hardwareMap.get(WebcamName .class, "Webcam 1"));
//
//        aprilTag = new AprilTagProcessor.Builder().build();
//        builder.addProcessor(aprilTag);
//
//
        waitForStart();
        sleep(3000);
//        while(aprilTag.getDetections().isEmpty()){}
//
//        findAprilTag();

        if (true) {
            Pose2d beginPose = new Pose2d(0, 0, 0);
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeToLinearHeading(new Vector2d(-24, -36), - Math.PI)
                        .strafeToLinearHeading(new Vector2d(-54, -36), - Math.PI)
                        .strafeToLinearHeading(new Vector2d(-24, 24), 3 * Math.PI / 4)
                        //.splineTo(new Vector2d(0, 60), Math.PI)
                        .build());
        }


    }
}


