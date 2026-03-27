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

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="goBilda Code V2", group="Development")
public class MainCodeV2 extends LinearOpMode {

    private DcMotorEx leftFrontDrive, leftBackDrive, rightBackDrive, rightFrontDrive;
    private DcMotorEx launcherDrive, shooterDrive;

    private double shooterPower = 0.80;
    private GoBildaPinpointDriver odo;
    private HuskyLens husky;

    // timers to prevent lag and handle button timing
    private ElapsedTime buttonTimer = new ElapsedTime();
    private ElapsedTime huskyTimer = new ElapsedTime();

    private int foundTagsCount = 0;

    @Override
    public void runOpMode() {
        // hardware mapping
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        husky = hardwareMap.get(HuskyLens.class, "huskylens");

        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        launcherDrive = hardwareMap.get(DcMotorEx.class, "ramp_drive");
        shooterDrive = hardwareMap.get(DcMotorEx.class, "shooter_drive");

        // odometry setup
        odo.setOffsets(-60, 175.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        // motor directions and braking
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // husky lens mode
        husky.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();
        buttonTimer.reset();
        huskyTimer.reset();

        while (opModeIsActive()) {
            // mecanum drive math
            double axial   = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double lf = axial + lateral + yaw;
            double rf = axial - lateral - yaw;
            double lb = axial - lateral + yaw;
            double rb = axial + lateral - yaw;

            // normalize motor power
            double max = Math.max(Math.abs(lf), Math.abs(rf));
            max = Math.max(max, Math.abs(lb));
            max = Math.max(max, Math.abs(rb));

            if (max > 1.0) {
                lf /= max; rf /= max; lb /= max; rb /= max;
            }

            // set power to motors
            leftFrontDrive.setPower(lf);
            rightFrontDrive.setPower(rf);
            leftBackDrive.setPower(lb);
            rightBackDrive.setPower(rb);

            // intake/launcher control
            if (gamepad1.triangle) {
                launcherDrive.setPower(1);
            } else if (gamepad1.x) {
                launcherDrive.setPower(-1);
            } else {
                launcherDrive.setPower(0);
            }

            // shooter control
            if (gamepad1.left_bumper) {
                shooterDrive.setPower(shooterPower);
            } else if (gamepad1.right_bumper) {
                shooterDrive.setPower(-shooterPower);
            } else {
                shooterDrive.setPower(0);
            }

            // change shooter power with dpad
            if (buttonTimer.seconds() > 0.2) {
                if (gamepad1.dpad_up) {
                    shooterPower += 0.05;
                    buttonTimer.reset();
                } else if (gamepad1.dpad_down) {
                    shooterPower -= 0.05;
                    buttonTimer.reset();
                }
            }

            // reset odometry position
            if (gamepad1.b) {
                odo.resetPosAndIMU();
            }

            // read husky lens every 100ms to stop lag
            if (huskyTimer.milliseconds() > 100) {
                foundTagsCount = husky.blocks().length;
                huskyTimer.reset();
            }

            // screen info
            telemetry.addData("shooter power", "%.2f", shooterPower);
            telemetry.addData("tags seen", foundTagsCount);
            telemetry.update();
        }
    }
}