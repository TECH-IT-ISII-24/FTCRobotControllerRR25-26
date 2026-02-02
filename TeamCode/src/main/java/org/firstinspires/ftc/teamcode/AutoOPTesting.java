package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetDeviceConfigurationForOpMode();
        pinpoint.resetPosAndIMU();
        pinpoint.recalibrateIMU();
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        sleep(3000);

        waitForStart();

        if (true) {
            Pose2d beginPose = new Pose2d(0, 0, 0);
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .strafeToLinearHeading(new Vector2d(-24, -36), - Math.PI)
                            .strafeToLinearHeading(new Vector2d(-54, -36), - Math.PI)
                            //.strafeToLinearHeading(new Vector2d(-24, 24), 3 * Math.PI / 4)
                            .build());
        }


    }
}