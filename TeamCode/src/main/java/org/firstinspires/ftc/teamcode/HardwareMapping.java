package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class HardwareMapping {

    static DcMotorEx upperSliderDrive = hardwareMap.get(DcMotorEx.class, "upper_slider_drive");
    static DcMotorEx lowerSliderDrive = hardwareMap.get(DcMotorEx.class, "lower_slider_drive");
    static DigitalChannel upperSliderSensor = hardwareMap.get(DigitalChannel.class, "upper_slider_sensor");
    static DigitalChannel lowerSliderSensor = hardwareMap.get(DigitalChannel.class, "lower_slider_sensor");
    static Servo upperRotationServo = hardwareMap.get(Servo.class, "upper_rotation_servo");
    static Servo lowerRotationServo = hardwareMap.get(Servo.class, "lower_rotation_servo");
    static Servo upperClosingServo = hardwareMap.get(Servo.class, "upper_closing_servo");
    static Servo lowerClosingServo = hardwareMap.get(Servo.class, "lower_closing_servo");

    static DcMotorEx leftFrontDrive = hardwareMap.get(DcMotorEx.class, "left_front_drive");
    static DcMotorEx leftBackDrive = hardwareMap.get(DcMotorEx.class, "left_back_drive");
    static DcMotorEx rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");
    static DcMotorEx rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");



}
