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
    double shooterPower = 0.50;

    @Override
    public void runOpMode() {
        Pose2d beginPose;
        MecanumDrive drive = null;

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

            telemetry.addData("Position ", (isBeginPoseBottom) ? "Bottom" : "Top");
            telemetry.addData("Team ", (isTeamBlue) ? "Blue" : "Red");
            telemetry.addData("Confirmed ", (found) ? "Yes" : "No");
            telemetry.update();

        } while (!found && !isStopRequested());

        waitForStart();

        if (!isTeamBlue) {
            if (isBeginPoseBottom) {
                beginPose = new Pose2d(63, 24, Math.PI);
                //BlueBottom(beginPose, new MecanumDrive(hardwareMap, beginPose));
            } else {
                beginPose = new Pose2d(-60, 39, 0);
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
        } while (getRuntime() < 5);
        return 48;
    }

    public void DriveToObelisk(Pose2d beginPose, MecanumDrive drive, boolean isTeamBlue) {
        int teamOffset = (!isTeamBlue) ? +12 : -12;
        double teamTangent = (!isTeamBlue) ? -Math.PI / 2 : Math.PI / 2 ;
        double teamRotation = (!isTeamBlue) ? 1.10 : 0.90;


        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .setTangent(Math.PI)
                        .splineToLinearHeading(new Pose2d(0, teamOffset, Math.PI * teamRotation), teamTangent)
                        //Lanciare palline pre-caricate
                        .build()

        );
        resetRuntime();
        int obeliskOffset = DecodeObelisk(husky);
        //obeliskOffset = 24;



        beginPose = new Pose2d(0, teamOffset, Math.PI * teamRotation);
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
                        .stopAndAdd(setDrive(shooterDrive, shooterPower))
                        .strafeToLinearHeading(new Vector2d(-12,-12), Math.PI * 1.15)
                        .waitSeconds(1)
                        .stopAndAdd(setDrive(rampDrive, 1))
                        .waitSeconds(3.25)
                        .stopAndAdd(setDrive(shooterDrive, 0))
                        .stopAndAdd(setDrive(rampDrive, 0))
                        //END of Set 1, Begin SPIKE collection
                        .setTangent(Math.PI)
                        .strafeToLinearHeading(FixedVector(35 - obeliskOffset, -12), Math.PI / 2)
                        .stopAndAdd(setDrive(rampDrive, 1.0))
                        .strafeToLinearHeading(FixedVector(35 - obeliskOffset, -60), Math.PI / 2, null, new ProfileAccelConstraint(-20, 25))
                        .stopAndAdd(setDrive(rampDrive, 0))
                        .stopAndAdd(setDrive(rampDrive, -0.1))
                        .waitSeconds(0.5)
                        .stopAndAdd(setDrive(rampDrive, 0))
                        //Ready flywheel for Set 2
                        .stopAndAdd(setDrive(shooterDrive, shooterPower))
                        .splineToLinearHeading(FixedPose(-8, -12, Math.PI * 1.15), Math.PI)
                        .stopAndAdd((setDrive(rampDrive, 1.0)))
                        .waitSeconds(3.25)
                        .stopAndAdd(setDrive(shooterDrive, 0))
                        .stopAndAdd(setDrive(rampDrive, 0))
                        .strafeTo(new Vector2d(-48, -24))
                        //End of Set 2
                        .build()
        );
    }

    public void RedAuto(Pose2d beginPose, MecanumDrive drive, int obeliskOffset) {
        DcMotorEx rampDrive = drive.rampDrive;
        DcMotorEx shooterDrive = drive.shooterDrive;

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        //Post-AUTO, ready to shoot pre-loaded
                        .stopAndAdd(setDrive(shooterDrive, shooterPower))
                        .strafeToLinearHeading(new Vector2d(-12,12), Math.PI * 0.87)
                        .waitSeconds(1)
                        .stopAndAdd(setDrive(rampDrive, 1))
                        .waitSeconds(3.25)
                        .stopAndAdd(setDrive(shooterDrive, 0))
                        .stopAndAdd(setDrive(rampDrive, 0))
                        //END of Set 1, Begin SPIKE collection
                        .setTangent(Math.PI)
                        .strafeToLinearHeading(FixedVector(35 - obeliskOffset, 12), -Math.PI / 2)
                        .stopAndAdd(setDrive(rampDrive, 1.0))
                        .strafeToLinearHeading(FixedVector(35 - obeliskOffset, 60), -Math.PI / 2, null, new ProfileAccelConstraint(-20, 25))
                        .stopAndAdd(setDrive(rampDrive, 0))
                        .stopAndAdd(setDrive(rampDrive, -0.1))
                        .waitSeconds(0.5)
                        .stopAndAdd(setDrive(rampDrive, 0))
                        //Ready flywheel for Set 2
                        .stopAndAdd(setDrive(shooterDrive, shooterPower))
                        .splineToLinearHeading(FixedPose(-8, 12, Math.PI * 0.87), Math.PI)
                        .stopAndAdd((setDrive(rampDrive, 1.0)))
                        .waitSeconds(3.25)
                        .stopAndAdd(setDrive(shooterDrive, 0))
                        .stopAndAdd(setDrive(rampDrive, 0))
                        //End of Set 2
//                        .strafeToLinearHeading(FixedVector(60, -32), Math.PI / 2)
//                        .stopAndAdd(setDrive(rampDrive, 1.0))
//                        .strafeToLinearHeading(FixedVector(60, -63), Math.PI / 2, null, new ProfileAccelConstraint(-20, 25))
//                        .stopAndAdd(setDrive(rampDrive, 0))
//                        .stopAndAdd(setDrive(rampDrive, -0.1))
//                        .waitSeconds(0.5)
//                        .stopAndAdd(setDrive(rampDrive, 0))
//                        .stopAndAdd(setDrive(shooterDrive, shooterPower))
//                        .setTangent(Math.PI / 2 )
//                        .splineToLinearHeading(new Pose2d(-8,-12, Math.PI * 0.70), 0 )
//                        .stopAndAdd(setDrive(rampDrive, 1))
//                        .waitSeconds(3.25)
                        .strafeTo(new Vector2d(-48, 24))
                        .build()
        );
    }
}

