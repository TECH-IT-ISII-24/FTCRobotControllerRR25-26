package org.firstinspires.ftc.teamcode.tuning;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Dictionary;
import java.util.Hashtable;
import java.util.List;



public final class SplineTest extends LinearOpMode {
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

        if (false) {
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
        aprilTagDict.put(11,new AprilTagInformation(-48,-72, 0));
        aprilTagDict.put(12,new AprilTagInformation(-72,0, 90));
        aprilTagDict.put(13,new AprilTagInformation(-48,+72, 180));
        aprilTagDict.put(14,new AprilTagInformation(+48,+72, 180));
        aprilTagDict.put(15,new AprilTagInformation(+72,+0, 270));
        aprilTagDict.put(16,new AprilTagInformation(+48,-72, 0));

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

        AprilTagInformation targetTagInfo = aprilTagDict.get(idTargetTag);

        if(targetTagInfo == null){
            throw new RuntimeException("Error #AP02 AprilTag ID is not valid");
        }
        else{
            double beginPoseX;
            double beginPoseY;
            boolean isAddition = checkTagOp(targetTagInfo.angle);
            boolean isXNegated = checkRelativeAng(targetTagInfo.angle);
            boolean areAxisInverted = checkAxisInversion(targetTagInfo.angle);


            xToTargetTag = (isXNegated)? xToTargetTag : -xToTargetTag;

            double supportVar = xToTargetTag;
            xToTargetTag = (areAxisInverted) ? yToTargetTag : xToTargetTag;
            yToTargetTag = (areAxisInverted) ? supportVar : yToTargetTag;

            beginPoseX = (isAddition) ? xToTargetTag + targetTagInfo.xPos : xToTargetTag - targetTagInfo.xPos;
            beginPoseY = (isAddition) ? yToTargetTag + targetTagInfo.yPos : yToTargetTag - targetTagInfo.yPos;

            telemetry.addLine("BeginPosX: " + beginPoseX);
            telemetry.addLine("BeginPosY: " + beginPoseY);
            telemetry.update();


            //To calculate the heading you need to add both angles together and subtract by 180.
            //TODO: normalize angle values so that they can be processed as radiant.
            //Pose2d beginPose = new Pose2d(beginPoseX, beginPoseY, 0);

        }

    }
    public boolean checkTagOp(int ang){
        //If the tag is within the negative zone, the operator
        //will be an addition due to the relativistic nature
        //of the points of reference (AprilTags orientation)
        return ang == 90 || ang == 0;
    }
    public boolean checkRelativeAng(int ang){
        //If the yaw is positive, and therefore the robot
        //is at the left of the AprilTag, the X vector
        //will need to be negated
        return ang <= 0;
    }

    public boolean checkAxisInversion(int ang){
        //If the robot is looking sideways, the relative
        //axis measurements and the global axis are flipped.
        //RelX will be GlobY and vice versa.
        return ang == 90 || ang == 270;
    }


}


class AprilTagInformation{
    int xPos;
    int yPos;
    int angle;

    public AprilTagInformation(int x, int y, int ang) {
         xPos = x;
         yPos = y;
         angle = ang;
    }
}
