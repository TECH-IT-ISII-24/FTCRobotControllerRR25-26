package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "HuskyTrackingTags", group = "Sensor")
public class HuskyTrackingTags extends LinearOpMode {

    private HuskyLens huskyLens;
    private ElapsedTime runtime = new ElapsedTime();

    // tag to track
    private final int TARGET_TAG = 1;

    @Override
    public void runOpMode() {

        // initialize huskylens from hardwareMap
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        telemetry.addData("Status", "Initializing HuskyLens...");
        telemetry.update();

        // check comms
        if (!huskyLens.knock()) {
            telemetry.addData("Error", "Cannot communicate with HuskyLens");
            telemetry.update();
            waitForStart();
            return;
        }

        // set mode to tag recognition
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        telemetry.addData("Status", "Tag Recognition Mode Enabled");
        telemetry.addData("Tracking Tag ID", TARGET_TAG);
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // get all detected blocks
            HuskyLens.Block[] blocks = huskyLens.blocks();

            // show infos
            telemetry.addData("Runtime", "%.2f s", runtime.seconds());
            telemetry.addData("Detected Blocks", blocks.length);

            boolean tagFound = false;

            for (int i = 0; i < blocks.length; i++) {
                HuskyLens.Block b = blocks[i];

                telemetry.addData("Block " + i,
                        "ID=" + b.id +
                        " X=" + b.x +
                        " Y=" + b.y +
                        " Width=" + b.width +
                        " Height=" + b.height);
                
                // check if tag is target
                if (b.id == TARGET_TAG) {
                    tagFound = true;
                    telemetry.addData("Locked Tag", "ID=" + b.id + " Center=(" + b.x + "," + b.y + ")");
                }
            }

            // if tag not found, then not visible
            if (!tagFound) {
                telemetry.addData("Locked Tag", "Not visible");
            }

            telemetry.update();
        }
    }
}