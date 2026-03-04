package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.roadrunner.Action;

@Autonomous(name="AutoOP", group="autonomous")
public class AutoOP extends LinearOpMode {
    public HuskyLens husky = null;
    public int obeliskOffset;
    double shooterPower = 0.55;

    @Override
    public void runOpMode() {
        Pose2d beginPose;
        MecanumDrive drive = null;
        //int obelisk = 2;
        boolean isTeamBlue = true;
        boolean isBeginPoseBottom = true;
        boolean found = false;

        husky = hardwareMap.get(HuskyLens.class, "huskylens");
        husky.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        if (!husky.knock()) telemetry.addData("Errore", "HuskyLens");


        //Leggi informazioni da controller per posizione iniziale e team.
        do {
            if (gamepad1.dpad_down) {
                isBeginPoseBottom = true;
            }
            if (gamepad1.dpad_up) {
                isBeginPoseBottom = false;

            }
            if (gamepad1.right_bumper) {
                isTeamBlue = true;
            }
            if (gamepad1.left_bumper) {
                isTeamBlue = false;
            }
            if (gamepad1.b) {
                found = true;
            }

            telemetry.addData("Position: ", (isBeginPoseBottom) ? "Bottom" : "Top");
            telemetry.addData("Team: ", (isTeamBlue) ? "Blue" : "Red");
            telemetry.addData("Confirmed: ", (found) ? "Yes" : "No");
            telemetry.update();

        } while (!found && !isStopRequested());

        waitForStart();

        if (isTeamBlue) {
            if (isBeginPoseBottom) {
                beginPose = new Pose2d(63, 24, Math.PI);
                //BlueBottom(beginPose, new MecanumDrive(hardwareMap, beginPose));
            } else {
                beginPose = new Pose2d(-63, 39, 0);
                //BlueTop(beginPose ,new MecanumDrive(hardwareMap, beginPose));
            }
        } else {
            if (isBeginPoseBottom) {
                beginPose = new Pose2d(63, -24, Math.PI);
                //RedBottom(beginPose, new MecanumDrive(hardwareMap, beginPose));
            } else {
                beginPose = new Pose2d(-63, -39, 0);
                //RedTop(beginPose, new MecanumDrive(hardwareMap, beginPose));
            }
        }

        DriveToObelisk(beginPose, new MecanumDrive(hardwareMap, beginPose), isTeamBlue);

    }

    public Vector2d FixedVector(double x, double y) {

        double newx = (x >= 0) ? x - 0 : x + 0;
        double newy = (y >= 0) ? y - 0 : y + 0;
        return new Vector2d(newx, newy);

    }

    public Pose2d FixedPose(double x, double y, double heading) {

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


    public int DecodeObelisk(HuskyLens husky) {
        do {
            HuskyLens.Block[] tags = husky.blocks();
            for (HuskyLens.Block tag : tags) {
                if (tag.id > 0 && tag.id < 4) {
                    return (tag.id - 1) * 24;
                }
            }
            sleep(500);
        } while (true);
    }

    public void DriveToObelisk(Pose2d beginPose, MecanumDrive drive, boolean isTeamBlue) {
        int teamOffset = (isTeamBlue) ? +12 : -12;
        double teamRotation = (isTeamBlue) ? Math.PI * ((double) 7 /6) : Math.PI * ((double) 5 /6);

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineToLinearHeading(new Pose2d(-20, teamOffset,teamRotation), beginPose.heading)
                        //Lanciare palline pre-caricate
                        .build()

        );

        int obeliskOffset = DecodeObelisk(husky);

        beginPose = new Pose2d(-20, teamOffset, teamRotation);
        if (isTeamBlue) {
            BlueAuto(beginPose, drive, obeliskOffset);
        } else {
            RedAuto(beginPose, drive, obeliskOffset);
        }
    }

    public void BlueAuto(Pose2d beginPose, MecanumDrive drive, int obeliskOffset) {
        DcMotorEx rampDrive = drive.rampDrive;
        DcMotorEx shooterDrive = drive.shooterDrive;

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .stopAndAdd(setDrive(shooterDrive, 1))
                        .waitSeconds(2)
                        // move to ball point
                        .stopAndAdd(setDrive(rampDrive, 1))
                        .waitSeconds(5)
                        .stopAndAdd(setDrive(shooterDrive, 1))
                        .stopAndAdd(setDrive(rampDrive, 1))

                        .waitSeconds(2)
                        .turnTo(Math.PI * ((double) 5/4))
                        .setTangent(Math.PI / 2)
                        .strafeToLinearHeading(FixedVector(32 - obeliskOffset, 24), -Math.PI / 2)


                        // load ball
                        .stopAndAdd(setDrive(rampDrive, 1.0))
                        .strafeToLinearHeading(FixedVector(32 - obeliskOffset, 63), -Math.PI / 2, null, new ProfileAccelConstraint(-25, 30))
                        //TODO potremmo usare azioni parallele..
                        //Principalmente nello spinup del launcher mentre ci spostiamo in posizione
                        //Le posizioni di lancio potrebbero cambiare in base a come è
                        //configurata la rampa.

                        .stopAndAdd(setDrive(rampDrive, 0))
                        .stopAndAdd(setDrive(rampDrive, -0.1))
                        .waitSeconds(0.5)
                        .stopAndAdd(setDrive(rampDrive, 0))

                        // move to goal
                        .setTangent(-Math.PI / 2)
                        .stopAndAdd(setDrive(shooterDrive, shooterPower))
                        .splineToLinearHeading(FixedPose(-8, +12, (5.0 / 4.0) * Math.PI), Math.PI)

                        // shoot ball
                        .stopAndAdd((setDrive(rampDrive, 1.0)))

                        // turn off all
                        .waitSeconds(5.0)
                        .stopAndAdd(setDrive(shooterDrive, 0))
                        .stopAndAdd(setDrive(rampDrive, 0))
                        .build()
        );
    }

    public void RedAuto(Pose2d beginPose, MecanumDrive drive, int obeliskOffset) {
        DcMotorEx rampDrive = drive.rampDrive;
        DcMotorEx shooterDrive = drive.shooterDrive;

        obeliskOffset = DecodeObelisk(husky);
        beginPose = new Pose2d(0, 0, Math.PI);

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .waitSeconds(1)
                        //.turnTo(Math.PI * ((double) 3/4))
                        .waitSeconds(1)
                        .setTangent(Math.PI)
                        //.strafeToLinearHeading(FixedVector(35, -24) , Math.PI/2)
                        //.strafeTo(FixedVector(57, -24))
                        .strafeToLinearHeading(FixedVector(32 - obeliskOffset, -24), Math.PI / 2)


                        .stopAndAdd(setDrive(rampDrive, 1.0))
                        .strafeToLinearHeading(FixedVector(32 - obeliskOffset, -63), Math.PI / 2, null, new ProfileAccelConstraint(-20, 25))
                        .stopAndAdd(setDrive(rampDrive, 0))
                        .stopAndAdd(setDrive(rampDrive, -0.1))
                        .waitSeconds(0.5)
                        .stopAndAdd(setDrive(rampDrive, 0))

                        // move to goal
                        //.setTangent(Math.PI / 2)
                        .stopAndAdd(setDrive(shooterDrive, shooterPower))
                        .waitSeconds(1)
                        .splineToLinearHeading(FixedPose(-8, -12, (5.0 / 4.0) * -Math.PI), Math.PI)

                        // shoot ball
                        .stopAndAdd((setDrive(rampDrive, 1.0)))

                        // turn off all
                        .waitSeconds(5.0)
                        .stopAndAdd(setDrive(shooterDrive, 0))
                        .stopAndAdd(setDrive(rampDrive, 0))
                        .build()
        );
    }
}

