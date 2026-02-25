package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.roadrunner.Action;

@Autonomous(name="AutoOPBlue", group="autonomous")
public class AutoOPBlue extends LinearOpMode {
    private DcMotorEx rampDrive = null;
    private DcMotorEx shooterDrive = null;

    @Override
    public void runOpMode() {
        // TO DO: CAMBIARE LA CONFIGURAZIONE DEL ROBOT PER LA RAMPDRIVE
        // TO DO: CAMBIARE LA CONFIGURAZIONE DEL ROBOT PER LA RAMPDRIVE
        // TO DO: CAMBIARE LA CONFIGURAZIONE DEL ROBOT PER LA RAMPDRIVE
        // TO DO: CAMBIARE LA CONFIGURAZIONE DEL ROBOT PER LA RAMPDRIVE
        // TO DO: CAMBIARE LA CONFIGURAZIONE DEL ROBOT PER LA RAMPDRIVE
        // TO DO: CAMBIARE LA CONFIGURAZIONE DEL ROBOT PER LA RAMPDRIVE
        // TO DO: CAMBIARE LA CONFIGURAZIONE DEL ROBOT PER LA RAMPDRIVE
        // TO DO: CAMBIARE LA CONFIGURAZIONE DEL ROBOT PER LA RAMPDRIVE
        // TO DO: CAMBIARE LA CONFIGURAZIONE DEL ROBOT PER LA RAMPDRIVE
        // TO DO: CAMBIARE LA CONFIGURAZIONE DEL ROBOT PER LA RAMPDRIVE

        //TODO: Spostare il mapping dei motori in MecanumDrive per unificarli

        rampDrive = hardwareMap.get(DcMotorEx.class, "ramp_drive");
        shooterDrive = hardwareMap.get(DcMotorEx.class, "shooter_drive");

        shooterDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rampDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Pose2d beginPose = new Pose2d(63, 24, Math.PI);
        //MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Pose2d beginPose = null;
        MecanumDrive drive = null;

        waitForStart();

        int obelisk = 2;
        boolean isTeamBlue = true;
        boolean isBeginPoseBottom = true;
        
        /* TO DO: LEGGERE OBELISK */
        if(isTeamBlue){
            if(isBeginPoseBottom){
                beginPose = new Pose2d(63, 24, Math.PI);
                BlueBottom(beginPose, new MecanumDrive(hardwareMap, beginPose));
            }
            else{
                beginPose = new Pose2d(-63, 39, Math.PI/2);
                BlueTop(beginPose ,new MecanumDrive(hardwareMap, beginPose));
            }
        }
        else{
            if(isBeginPoseBottom){
                beginPose = new Pose2d(63, -39, Math.PI);
                RedBottom(beginPose, new MecanumDrive(hardwareMap, beginPose));
            }
            else{
                beginPose = new Pose2d(-63, -39, Math.PI);
                RedTop(beginPose, new MecanumDrive(hardwareMap, beginPose));
            }
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

    public Action setRamp(double power) {
        return packet -> {
            rampDrive.setPower(power);
            return false;
        };
    }
    
    public Action setShooter(double power) {
        return packet -> {
            shooterDrive.setPower(power);
            return false;
        };
    }

    public void BlueBottom(Pose2d beginPose, MecanumDrive drive){
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        // move to ball point
                        //.stopAndAdd(setLauncher(1.0))
                        .setTangent(Math.PI / 2)
                        .strafeToLinearHeading(FixedVector(35, 24), -Math.PI / 2)

                        // load ball
                        .stopAndAdd(setRamp(1.0))
                        .strafeToLinearHeading(FixedVector(35, 63), -Math.PI / 2)
                        .stopAndAdd(setRamp(0))

                        // move to goal
                        .setTangent(-Math.PI / 2)
                        .afterTime(0, setShooter(1.0))
                        .splineToLinearHeading(FixedPose(-24, -24, (5.0 / 4.0) * Math.PI), Math.PI)

                        // shoot ball
                        .stopAndAdd(setRamp(1.0))
                        .waitSeconds(2.0)

                        // turn off all
                        .stopAndAdd(setRamp(0))
                        .stopAndAdd(setShooter(0))
                        .build()
        );
    }

    public void BlueTop(Pose2d beginPose, MecanumDrive drive){
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        // move to ball point
                        //.stopAndAdd(setLauncher(1.0))
                        .setTangent(-Math.PI / 2)
                        .splineToLinearHeading(FixedPose(35, 24,Math.PI/2) , Math.PI/2)

                        // load ball
                        .stopAndAdd(setRamp(1.0))
                        .strafeToLinearHeading(FixedVector(35, 63), -Math.PI / 2)
                        .stopAndAdd(setRamp(0))

                        // move to goal
                        .setTangent(-Math.PI / 2)
                        .afterTime(0, setShooter(1.0))
                        .splineToLinearHeading(FixedPose(-24, -24, (5.0 / 4.0) * Math.PI), Math.PI)

                        // shoot ball
                        .stopAndAdd(setRamp(1.0))
                        .waitSeconds(2.0)

                        // turn off all
                        .stopAndAdd(setRamp(0))
                        .stopAndAdd(setShooter(0))
                        .build()
        );
    }

    public void RedBottom(Pose2d beginPose, MecanumDrive drive){
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                    .setTangent(-Math.PI / 2)
                    .strafeToLinearHeading(FixedVector(35, -24) , -Math.PI/2)

                    // load ball
                    .stopAndAdd(setRamp(1.0))
                    .strafeToLinearHeading(FixedVector(35, -63), -Math.PI / 2)
                    .stopAndAdd(setRamp(0))

                    // move to goal
                    .setTangent(Math.PI / 2)
                    .afterTime(0, setShooter(1.0))

                    .splineToLinearHeading(FixedPose(-24, 24, -(5.0 / 4.0) * Math.PI), Math.PI)

                    // shoot ball
                    .stopAndAdd(setRamp(1.0))
                    .waitSeconds(2.0)

                    // turn off all
                    .stopAndAdd(setRamp(0))
                    .stopAndAdd(setShooter(0))
                    .build());
    }

    public void RedTop(Pose2d beginPose, MecanumDrive drive){
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                    .setTangent(0)
                    .splineToLinearHeading(FixedPose(35, -24, -Math.PI/2) , -Math.PI/2)

                    // load ball
                    .stopAndAdd(setRamp(1.0))
                    .strafeToLinearHeading(FixedVector(35, -63), -Math.PI / 2)
                    .stopAndAdd(setRamp(0))

                    // move to goal
                    .setTangent(Math.PI / 2)
                    .afterTime(0, setShooter(1.0))

                    .splineToLinearHeading(FixedPose(-24, 24, -(5.0 / 4.0) * Math.PI), Math.PI)

                    // shoot ball
                    .stopAndAdd(setRamp(1.0))
                    .waitSeconds(2.0)

                    // turn off all
                    .stopAndAdd(setRamp(0))
                    .stopAndAdd(setShooter(0))
                    .build());
    }


}