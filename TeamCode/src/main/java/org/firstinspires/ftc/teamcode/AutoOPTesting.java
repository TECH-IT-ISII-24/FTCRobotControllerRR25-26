package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Dictionary;
import java.util.Hashtable;
import java.util.List;


@Autonomous(name = "AutoOPTesting", group = "Development")
public final class AutoOPTesting extends LinearOpMode {

    public GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() throws InterruptedException {



        waitForStart();


        Pose2d beginPose = new Pose2d(48, -63, Math.PI/2);
        sleep(3000);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .setTangent(Math.PI/ 2)
                            .strafeToLinearHeading(new Vector2d(48, 24) , Math.PI)
                            .splineToLinearHeading(new Pose2d(24, 48, -Math.PI/2), Math.PI)
                            //.strafeToLinearHeading(new Vector2d(-35, -36), - Math.PI)
                            //.strafeToLinearHeading(new Vector2d(-24, 4), 3 * Math.PI / 4)
                            .build());
    }
    public Vector2d FixedVector (double x,double y){

        double newx = (x >= 0) ? x - 4 : x + 4;
        double newy = (y >= 0) ? y - 4 : y + 4;
        return new Vector2d(newx, newy);

    }
    public Pose2d FixedPose (double x,double y, double heading){

        double newx = (x >= 0) ? x - 4 : x + 4;
        double newy = (y >= 0) ? y - 4 : y + 4;
        return new Pose2d(newx, newy, heading);

    }
}

