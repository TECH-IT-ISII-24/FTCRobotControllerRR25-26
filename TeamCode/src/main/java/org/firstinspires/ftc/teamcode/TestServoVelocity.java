package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TestServoVelocity", group="FINAL")
public class TestServoVelocity extends LinearOpMode {

    private DcMotor servo = null;

    public void runOpMode() {
        servo = hardwareMap.get(DcMotor.class, "servotest");
        servo.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.dpad_up) {
                servo.setPower(1);
            } else {
                servo.setPower(0);
            }
        }
    }
}
