/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="MainCode", group="FINAL")
public class MainCode extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor extensionScoopDrive = null;


    private DcMotor extensionArmDrive = null;



    private DigitalChannel extensionArmSensor = null;

    private Servo rotationArmServo = null;

    private DigitalChannel extensionScoopSensor = null;

    private Servo clawOpeningServo = null;

    private Servo clawRotationServo = null;

    private Servo armOpeningServo = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        extensionArmDrive = hardwareMap.get(DcMotor.class, "extension_arm_drive");
        extensionScoopDrive = hardwareMap.get(DcMotor.class, "extension_scoop_drive");
        clawOpeningServo = hardwareMap.get(Servo.class, "claw_opening_servo");
        clawRotationServo = hardwareMap.get(Servo.class, "claw_rotation_servo");
        armOpeningServo = hardwareMap.get(Servo.class, "arm_opening_servo");
        rotationArmServo = hardwareMap.get(Servo.class, "rotation_arm_servo");


        clawOpeningServo.scaleRange(0, 0.32);
        clawOpeningServo.setPosition(0);
        clawRotationServo.scaleRange(0,0.75);
        clawRotationServo.setPosition(0);
        armOpeningServo.scaleRange(0,1);
        armOpeningServo.setPosition(1);
        rotationArmServo.scaleRange(0,0.96);
        rotationArmServo.setPosition(0);

        double highCorrectionRange = 0.005;
        double lowCorrectionRange = 0;
        boolean isHorizontalResetting = false;
        boolean isDiagonalResetting = false;




        extensionArmSensor = hardwareMap.get(DigitalChannel.class, "arm_sensor");
        extensionScoopSensor = hardwareMap.get(DigitalChannel.class, "scoop_sensor");



        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Resetting Positions...");
        telemetry.update();




        //rotationDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        extensionArmDrive.setPower(0);
        extensionArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionArmDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "Hot and ready!");
        extensionScoopDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionScoopDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rotationArmDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.update();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;



            if(gamepad1.dpad_down){
                //Arm
                if(extensionArmSensor.getState()) {
                    extensionArmDrive.setPower(0.85);
                }
                else{
                    extensionArmDrive.setPower(0);
                    extensionArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extensionArmDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }


            }
            else if(gamepad1.dpad_up){
                if(extensionArmDrive.getCurrentPosition() > -3100)
                    extensionArmDrive.setPower(-0.75);
                else{
                    extensionArmDrive.setPower(0);
                }
            }
            else{
                extensionArmDrive.setPower(0);
            }

            if(gamepad1.a){
                //Arm
                if(extensionScoopSensor.getState()) {
                    if (extensionScoopDrive.getCurrentPosition() > -250) {
                        extensionScoopDrive.setPower(0.35);
                    }
                    else {
                        extensionScoopDrive.setPower(0.60);
                    }
                }
                else{
                    extensionScoopDrive.setPower(0);
                    extensionScoopDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extensionScoopDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }


            }
            else if(gamepad1.y){
                if(extensionScoopDrive.getCurrentPosition() > -540)
                    extensionScoopDrive.setPower(-0.5);
                else{
                    extensionScoopDrive.setPower(0);
                }

            }
            else{
                extensionScoopDrive.setPower(0);
            }

            if(gamepad2.left_stick_y > 0.65){
                highCorrectionRange = highCorrectionRange + 0.0005;
                if(highCorrectionRange > 0.039){
                    highCorrectionRange = 0.039;
                }
                rotationArmServo.scaleRange(0,0.96 + highCorrectionRange);
                rotationArmServo.setPosition(1);
            }
            else if (gamepad2.left_stick_y < -0.65){
                highCorrectionRange = highCorrectionRange - 0.0005;

                if(highCorrectionRange < -0.039){
                    highCorrectionRange = -0.039;
                }
                rotationArmServo.scaleRange(0,0.96 + highCorrectionRange);
                rotationArmServo.setPosition(0.975);
            }

            if(gamepad2.right_stick_y < -0.65){
                lowCorrectionRange = lowCorrectionRange + 0.0005;
                if(lowCorrectionRange > 0.039){
                    lowCorrectionRange = 0.039;
                }

                clawRotationServo.scaleRange(0,0.75 + lowCorrectionRange);
                clawRotationServo.setPosition(0.975);
            }
            else if (gamepad2.right_stick_y > 0.65){
                lowCorrectionRange = lowCorrectionRange - 0.0005;

                if(lowCorrectionRange < -0.039){
                    lowCorrectionRange = -0.039;
                }
                clawRotationServo.scaleRange(0,0.75 + lowCorrectionRange);
                clawRotationServo.setPosition(1);
            }

            if(gamepad1.left_bumper){
                isDiagonalResetting = true;
                extensionArmDrive.setPower(0.5);
            }
            if(!extensionArmSensor.getState() && isDiagonalResetting){
                extensionArmDrive.setPower(0);
                isDiagonalResetting = false;
            }

            if(gamepad1.right_bumper){
                isHorizontalResetting = true;
                extensionScoopDrive.setPower(0.5);
            }
            if(!extensionScoopSensor.getState() && isHorizontalResetting){
                extensionScoopDrive.setPower(0);
                isHorizontalResetting = false;

            }





            if(gamepad2.dpad_down){
                clawRotationServo.setPosition(0);
            }
            else if(gamepad2.dpad_up){
                clawRotationServo.setPosition(0.95);
            }
            if(gamepad2.left_bumper){
                clawOpeningServo.setPosition(0);
            }
            else if(gamepad2.right_bumper){
                clawOpeningServo.setPosition(1);
            }
            if(gamepad2.dpad_left){
                armOpeningServo.setPosition(0);
            }
            else if(gamepad2.dpad_right){
                armOpeningServo.setPosition(1);
            }
            if(gamepad2.y){
                rotationArmServo.setPosition(0);
            }
            else if(gamepad2.a){
                rotationArmServo.setPosition(0.975);
            }
            else if(gamepad2.x){
                rotationArmServo.setPosition(0.5);

            }
            else if(gamepad2.b){
                rotationArmServo.setPosition(0.90);
            }


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x; //axial

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Front left/Right", "%4.2f, %4.2f ", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f ", leftBackPower, rightBackPower);
            telemetry.addData("Arm Sensor", "Extended? " + extensionArmSensor.getState());
            telemetry.addData("Scoop Sensor", "Extended? " + extensionScoopSensor.getState());
            telemetry.addData("Arm Extension Ticks:", "Ticks: " + extensionArmDrive.getCurrentPosition());
            telemetry.addData("Scoop Ticks:", "Ticks:" + extensionScoopDrive.getCurrentPosition());
            telemetry.addData("Claw Opening Position:", "Value: " + clawRotationServo.getPosition());
            telemetry.addData("Arm Rotation Ticks:", "Value: " + rotationArmServo.getPosition());
            telemetry.addData("Arm correction:",  "%4.4f", highCorrectionRange );
            telemetry.addData("Rotation correction:",  "%4.4f", lowCorrectionRange );

            telemetry.update();
        }


    }}


