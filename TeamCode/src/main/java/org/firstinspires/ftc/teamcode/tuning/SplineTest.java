package org.firstinspires.ftc.teamcode.tuning;
import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.ImuParameters;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import static java.lang.Math.sqrt;

import java.util.Dictionary;
import java.util.Hashtable;
import java.util.List;



public final class SplineTest extends LinearOpMode {
    private AprilTagProcessor aprilTag;





    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();

        findAprilTag();

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            Pose2d beginPose = new Pose2d(0, 0, 0);
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
                        .splineTo(new Vector2d(0, 60), Math.PI)
                        .build());
        }
    }

    public void findAprilTag(){

        //TODO: Method this shit man wth
        Dictionary<Integer, AprilTagInformation> aprilTagDict = new Hashtable<>();
        aprilTagDict.put(11,new AprilTagInformation(-48,-72, false));
        aprilTagDict.put(12,new AprilTagInformation(-72,0, true));
        aprilTagDict.put(13,new AprilTagInformation(-48,+72, false));
        aprilTagDict.put(14,new AprilTagInformation(+48,+72, false));
        aprilTagDict.put(15,new AprilTagInformation(+72,+0, true));
        aprilTagDict.put(16,new AprilTagInformation(+48,-72, false));

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        AprilTagDetection TargetTag;
        try{
            TargetTag = currentDetections.get(0);
        }
        catch(Exception e){
            throw new RuntimeException("Error #AP00 No AprilTag Found");
        }

        double xToTargetTag;
        double yToTargetTag;
        double idTargetTag;

        if(TargetTag.metadata == null) {

            throw new RuntimeException("Error #AP01 AprilTag Metadata is Null");
        }
        else{
            AprilTagPoseFtc TargetTagPos = TargetTag.ftcPose;
            xToTargetTag = TargetTagPos.x;
            yToTargetTag = TargetTagPos.y;
            idTargetTag = TargetTag.id;
        }


        if(aprilTagDict.get(idTargetTag) == null){
            throw new RuntimeException("Error #AP02 AprilTag ID is not valid");
        }
        else{

        }

    }


}

class AprilTagInformation{
    int xPos;
    int yPos;
    boolean invertXY;

    public AprilTagInformation(int x, int y, boolean invert) {
         xPos = x;
         yPos = y;
         invertXY = invert;
    }
}
