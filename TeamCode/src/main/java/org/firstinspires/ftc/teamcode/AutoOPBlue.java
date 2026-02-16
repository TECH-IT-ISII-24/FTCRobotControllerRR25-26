package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="AutoOPBlue", group="autonomous")
public class AutoOPBlue extends LinearOpMode {

    private DcMotorEx leftFrontDrive, leftBackDrive, rightBackDrive, rightFrontDrive;
    private GoBildaPinpointDriver odo;
    private static final double IN_TO_MM = 25.4;

    @Override
    public void runOpMode() {
        // hardware init
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(60.0, -175.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // starting position selection
        int startCase = 1;
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) startCase = 1;
            if (gamepad1.b) startCase = 2;
            telemetry.addData("case", startCase);
            telemetry.update();
        }

        if (startCase == 1) {
            odo.setPosition(new Pose2D(DistanceUnit.MM, -24 * IN_TO_MM, -63 * IN_TO_MM, AngleUnit.DEGREES, 0));
        } else {
            odo.setPosition(new Pose2D(DistanceUnit.MM, 39 * IN_TO_MM, 63 * IN_TO_MM, AngleUnit.DEGREES, 180));
        }

        waitForStart();

        // movement sequence
        driveToPoint(-24, 24, -45); // shootpoint
        sleep(800);

        driveToPoint(-24, 12, -90); // line 3
        driveToPoint(-24, 24, -45);

        driveToPoint(-24, -12, -90); // line 2
        driveToPoint(-24, 24, -45);

        driveToPoint(-24, -36, -90); // line 1
        driveToPoint(-24, 24, -45);

        driveToPoint(31, -39, 0); // endpos


    }

    public void driveToPoint(double targetX, double targetY, double targetDegH) {

        while (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();

            double xError = targetX - pos.getX(DistanceUnit.MM);
            double yError = targetY - pos.getY(DistanceUnit.MM);
            double hError = targetDegH - pos.getHeading(AngleUnit.DEGREES);

            // check if target reached
            if (Math.hypot(xError, yError) < 25 && Math.abs(hError) < 2) {
                stopMotors();
                break;
            }

            // simple p-loop
            double kp = 0.0035;
            double kt = 0.018;

            // field centric calculation
            double headingRad = pos.getHeading(AngleUnit.RADIANS);
            double rotX = xError * Math.cos(-headingRad) - yError * Math.sin(-headingRad);
            double rotY = xError * Math.sin(-headingRad) + yError * Math.cos(-headingRad);

            double axial = rotX * kp;
            double lateral = rotY * kp;
            double yaw = hError * kt;

            double max = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1.0);
            leftFrontDrive.setPower((axial + lateral + yaw) / max);
            rightFrontDrive.setPower((axial - lateral - yaw) / max);
            leftBackDrive.setPower((axial - lateral + yaw) / max);
            rightBackDrive.setPower((axial + lateral - yaw) / max);
        }
    }

    private void stopMotors() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}