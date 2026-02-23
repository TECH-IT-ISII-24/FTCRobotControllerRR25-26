package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="AutoOPBlue", group="autonomous")
public class AutoOPBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();

        Pose2d beginPose = new Pose2d(63, 24, Math.PI);
        sleep(1000);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        int obelisk = 2;

        switch (obelisk) {
            case 2:
                Actions.runBlocking(
                        drive.actionBuilder(beginPose)
                                .setTangent(Math.PI/ 2)
                                .strafeToLinearHeading(FixedVector(35, 24) , Math.PI / 2)
                                .strafeToLinearHeading(FixedVector(35, 63), Math.PI / 2)
                                .setTangent(-Math.PI / 2)
                                .splineToLinearHeading(FixedPose(-24, -24, ((double) 5 / 4) * Math.PI), Math.PI)
                                .build());
        }
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