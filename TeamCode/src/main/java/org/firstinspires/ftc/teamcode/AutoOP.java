package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.roadrunner.Action;

@Autonomous(name="AutoOP", group="autonomous")
public class AutoOP extends LinearOpMode {

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


        Pose2d beginPose;
        MecanumDrive drive = null;
        int obelisk = 2;
        boolean isTeamBlue = true;
        boolean isBeginPoseBottom = true;
        boolean found = false;

        //Leggi informazioni da controller per posizione iniziale e team.
        do{
            if(gamepad1.dpad_down){
                isBeginPoseBottom = true;
            }
            if(gamepad1.dpad_up){
                isBeginPoseBottom = false;

            }
            if(gamepad1.right_bumper){
                isTeamBlue = true;
            }
            if(gamepad1.left_bumper){
                isTeamBlue = false;
            }
            if(gamepad1.b){
                found = true;
            }

            telemetry.addData("Position: ", (isBeginPoseBottom) ? "Bottom" : "Top" );
            telemetry.addData("Team: ", (isTeamBlue) ? "Blue" : "Red" );
            telemetry.addData("Confirmed: ", (found) ? "Yes" : "No" );
            telemetry.update();

        }while(!found);

        waitForStart();



        
        /* TO DO: LEGGERE OBELISK */
        if(isTeamBlue){
            if(isBeginPoseBottom){
                beginPose = new Pose2d(63, 24, Math.PI);
                BlueBottom(beginPose, new MecanumDrive(hardwareMap, beginPose));
            }
            else{
                beginPose = new Pose2d(-63, 39, 0);
                BlueTop(beginPose ,new MecanumDrive(hardwareMap, beginPose));
            }
        }
        else{
            if(isBeginPoseBottom){
                beginPose = new Pose2d(63, -24, Math.PI);
                RedBottom(beginPose, new MecanumDrive(hardwareMap, beginPose));
            }
            else{
                beginPose = new Pose2d(-63, -39, 0);
                RedTop(beginPose, new MecanumDrive(hardwareMap, beginPose));
            }
        }

    }

    public Vector2d FixedVector (double x,double y){

        double newx = (x >= 0) ? x - 0 : x + 0;
        double newy = (y >= 0) ? y - 0 : y + 0;
        return new Vector2d(newx, newy);

    }
    public Pose2d FixedPose (double x,double y, double heading){

        double newx = (x >= 0) ? x - 0 : x + 0;
        double newy = (y >= 0) ? y - 0 : y + 0;
        return new Pose2d(newx, newy, heading);

    }

    public Action setDrive(DcMotorEx motor, double power) {
        return packet -> {
            motor.setPower(power);
            return false;
        };
    }

    public void BlueBottom(Pose2d beginPose, MecanumDrive drive){
        DcMotorEx rampDrive = drive.rampDrive;
        DcMotorEx shooterDrive = drive.shooterDrive;
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        // move to ball point
                        //.stopAndAdd(setLauncher(1.0))
                        .setTangent(Math.PI / 2)
                        .strafeToLinearHeading(FixedVector(39, 24), -Math.PI / 2)

                        // load ball
                        .stopAndAdd(setDrive(rampDrive,1.0))
                        .strafeToLinearHeading(FixedVector(39, 63), -Math.PI / 2, null, new ProfileAccelConstraint(-25, 30))
                        //TODO potremmo usare azioni parallele..
                        //Principalmente nello spinup del launcher mentre ci spostiamo in posizione
                        //Le posizioni di lancio potrebbero cambiare in base a come è
                        //configurata la rampa.

                        .stopAndAdd(setDrive(rampDrive, 0))

                        // move to goal
                        .setTangent(-Math.PI / 2)
                        .stopAndAdd(setDrive(shooterDrive, 1.0))
                        .splineToLinearHeading(FixedPose(-6, -6, (5.0 / 4.0) * Math.PI), Math.PI)

                        // shoot ball
                        .stopAndAdd((setDrive(rampDrive, 1.0 )))

                        // turn off all
                        .waitSeconds(5.0)
                        .stopAndAdd(setDrive(shooterDrive,0))
                        .stopAndAdd(setDrive(rampDrive,0))
                        .build()
        );
    }

    public void BlueTop(Pose2d beginPose, MecanumDrive drive){
        DcMotorEx rampDrive = drive.rampDrive;
        DcMotorEx shooterDrive = drive.shooterDrive;
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        // move to ball point
                        //.stopAndAdd(setLauncher(1.0))
                        .setTangent(-Math.PI / 3)
                        .splineToLinearHeading(FixedPose(35, 24,-Math.PI/2) , Math.PI/2)

                        // load ball
                        .stopAndAdd(setDrive(rampDrive,1.0))
                        .strafeToLinearHeading(FixedVector(35, 63), -Math.PI / 2, null, new ProfileAccelConstraint(-20, 20))
                        .stopAndAdd(setDrive(rampDrive,0))

                        // move to goal
                        .setTangent(-Math.PI / 2)
                        .stopAndAdd(setDrive(shooterDrive, 1.0))
                        .splineToLinearHeading(FixedPose(6, -6, (5.0 / 4.0) * Math.PI), Math.PI)

                        // shoot ball
                        .stopAndAdd((setDrive(rampDrive, 1.0 )))

                        // turn off all
                        .waitSeconds(5.0)
                        .stopAndAdd(setDrive(shooterDrive,0))
                        .stopAndAdd(setDrive(rampDrive,0))
                        .build()
        );
    }

    public void RedBottom(Pose2d beginPose, MecanumDrive drive){
        DcMotorEx rampDrive = drive.rampDrive;
        DcMotorEx shooterDrive = drive.shooterDrive;
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                    .setTangent(-Math.PI / 1.5)
                    //.strafeToLinearHeading(FixedVector(35, -24) , Math.PI/2)
//                    .strafeTo(FixedVector(57, -24))
//                    .splineToLinearHeading(FixedPose(35, -24, Math.PI/2) , -Math.PI / 2)
//
//                        //load ball
//                    .stopAndAdd(setDrive(rampDrive,1.0))
//                    .strafeToLinearHeading(FixedVector(35, -63), Math.PI / 2, null, new ProfileAccelConstraint(-20, 25))
//                        .stopAndAdd(setDrive(rampDrive,0))
//
//
//                    // move to goal
//                    .setTangent(Math.PI / 2)
                    .stopAndAdd(setDrive(shooterDrive, 1.0))
                        .waitSeconds(1)
                    .splineToLinearHeading(FixedPose(-8, 4, (6.0 / 5.0) * -Math.PI), Math.PI)

                    // shoot ball
                    .stopAndAdd((setDrive(rampDrive, 1.0 )))

                    // turn off all
                    .waitSeconds(5.0)
                    .stopAndAdd(setDrive(shooterDrive,0))
                    .stopAndAdd(setDrive(rampDrive,0))
                    .build()
        );
    }

    public void RedTop(Pose2d beginPose, MecanumDrive drive){
        DcMotorEx rampDrive = drive.rampDrive;
        DcMotorEx shooterDrive = drive.shooterDrive;
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                    .setTangent(0)
                    .splineToLinearHeading(FixedPose(35, -24, Math.PI/2) , -Math.PI/2)

                    // load ball
                    .stopAndAdd(setDrive(rampDrive,1.0))
                    .strafeToLinearHeading(FixedVector(35, -63), Math.PI / 2, null, new ProfileAccelConstraint(-20, 25))
                    .stopAndAdd(setDrive(rampDrive,0))

                        // move to goal
                        .setTangent(Math.PI / 2)
                        .stopAndAdd(setDrive(shooterDrive, 1.0))
                        .splineToLinearHeading(FixedPose(-6, 6, (5.0 / 4.0) * -Math.PI), Math.PI)

                        // shoot ball
                        .stopAndAdd((setDrive(rampDrive, 1.0 )))

                        // turn off all
                        .waitSeconds(5.0)
                        .stopAndAdd(setDrive(shooterDrive,0))
                        .stopAndAdd(setDrive(rampDrive,0))
                        .build()
        );
    }
}