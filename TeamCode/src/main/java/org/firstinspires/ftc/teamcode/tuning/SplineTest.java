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
        builder.setCamera(hardwareMap.get(WebcamName .class, "Webcam 1"));

        //Initialize apriltag processor with Webcam 1
        aprilTag = new AprilTagProcessor.Builder().build();
        builder.addProcessor(aprilTag);


        waitForStart();
        sleep(3000);
        AprilTagDetection foundTag;
        //Wait until you can see a tag, then save it and start position calculation
        while(true){
            if (!aprilTag.getDetections().isEmpty()){
                foundTag = aprilTag.getDetections().get(0);
                break;
            }

        }
        //Call function to find intial pose to begin pathing
        Pose2d beginPose = findAprilTag(foundTag);

        if (true) {
            //Run trajectory
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
                        .splineTo(new Vector2d(0, 60), Math.PI)
                        .build());
        }


    }

    public Pose2d findAprilTag(AprilTagDetection TargetTag){

        //All known apriltags in 24-25
        Dictionary<Integer, AprilTagInformation> aprilTagDict = new Hashtable<>();
        aprilTagDict.put(11,new AprilTagInformation(-48,-72, 0));
        aprilTagDict.put(12,new AprilTagInformation(-72,0, 90));
        aprilTagDict.put(13,new AprilTagInformation(-48,+72, 180));
        aprilTagDict.put(14,new AprilTagInformation(+48,+72, 180));
        aprilTagDict.put(15,new AprilTagInformation(+72,+0, 270));
        aprilTagDict.put(16,new AprilTagInformation(+48,-72, 0));

        //Init variables for distance from detected tag
        double xToTargetTag;
        double yToTargetTag;
        double idTargetTag;
        double angToTargetTag;

        //Throw exception if tag metadata is not found is found.
        if(TargetTag.metadata == null) {

            throw new RuntimeException("Error #AP01 AprilTag Metadata is Null");
        }
        else{

            AprilTagPoseFtc TargetTagPos = TargetTag.ftcPose;
            xToTargetTag = TargetTagPos.x;
            yToTargetTag = TargetTagPos.y;
            angToTargetTag = TargetTagPos.yaw;
            idTargetTag = TargetTag.id;
        }
        //Get global coordinates of the found tag
        AprilTagInformation targetTagInfo = aprilTagDict.get(idTargetTag);


        //If the tag found it not a known one, throw an exception
        if(targetTagInfo == null){
            throw new RuntimeException("Error #AP02 AprilTag ID is not valid");
        }
        else{
            //Init variables for robot position
            double beginPoseX;
            double beginPoseY;
            double beginPoseAng;
            //Orientation checks. Look at functions for explaination
            boolean isAddition = checkTagOp(targetTagInfo.angle);
            boolean isXNegated = checkRelativeAng(targetTagInfo.angle);
            boolean areAxisInverted = checkAxisInversion(targetTagInfo.angle);

            //Assignment based on ori checks.
            xToTargetTag = (isXNegated)? xToTargetTag : -xToTargetTag;

            double supportVar = xToTargetTag;
            xToTargetTag = (areAxisInverted) ? yToTargetTag : xToTargetTag;
            yToTargetTag = (areAxisInverted) ? supportVar : yToTargetTag;

            beginPoseX = (isAddition) ? xToTargetTag + targetTagInfo.xPos : xToTargetTag - targetTagInfo.xPos;
            beginPoseY = (isAddition) ? yToTargetTag + targetTagInfo.yPos : yToTargetTag - targetTagInfo.yPos;



            beginPoseAng = (angToTargetTag * -1) + targetTagInfo.angle - 180;
            if (beginPoseAng < 0)
            {
                beginPoseAng += 360;
            }

            //Add to telemetry
            telemetry.addLine("BeginPosX: " + beginPoseX);
            telemetry.addLine("BeginPosY: " + beginPoseY);
            telemetry.addLine("BeginPosY: " + beginPoseY);
            telemetry.update();

            beginPoseAng = Math.toRadians(beginPoseAng);
            //Return the pose to the trajectory builder.
            return new Pose2d(beginPoseX,beginPoseY,beginPoseAng);

        }

    }
    public boolean checkTagOp(int ang){
        //If the tag is within the negative zone, the operator
        //will be an addition due to the relativistic nature
        //of the points of reference (AprilTags orientation)
        //This is analyzing the DICT information, NOT
        //The Detector metadata.
        //
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
