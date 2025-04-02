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

    public DcMotorEx upperSliderDrive = hardwareMap.get(DcMotorEx.class, "upper_slider_drive");
    public DcMotorEx lowerSliderDrive = hardwareMap.get(DcMotorEx.class, "lower_slider_drive");
    public DigitalChannel upperSliderSensor = hardwareMap.get(DigitalChannel.class, "upper_slider_sensor");
    public DigitalChannel lowerSliderSensor = hardwareMap.get(DigitalChannel.class, "lower_slider_sensor");
    public Servo upperRotationServo = hardwareMap.get(Servo.class, "upper_rotation_servo");
    public Servo lowerRotationServo = hardwareMap.get(Servo.class, "lower_rotation_servo");
    public Servo upperClosingServo = hardwareMap.get(Servo.class, "upper_closing_servo");
    public Servo lowerClosingServo = hardwareMap.get(Servo.class, "lower_closing_servo");

    public DcMotorEx leftFrontDrive = hardwareMap.get(DcMotorEx.class, "left_front_drive");
    public DcMotorEx leftBackDrive = hardwareMap.get(DcMotorEx.class, "left_back_drive");
    public DcMotorEx rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");
    public DcMotorEx rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");


    //TODO: First declare motor vars as null, otherwise you get a null pointer exception


}
