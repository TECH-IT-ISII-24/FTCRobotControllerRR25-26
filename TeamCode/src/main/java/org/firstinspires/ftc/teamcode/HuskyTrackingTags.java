package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

@TeleOp(name = "HuskyTrackingTags", group = "Sensor")
public class HuskyTrackingTags extends LinearOpMode {

    private HuskyLens huskyLens;
    private ElapsedTime runtime = new ElapsedTime();
    public List<Boolean> sequenza = new LinkedList<>();
    //VERDE == TRUE VIOLA == FALSE

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

                /*
                telemetry.addData("Block " + i,
                        "ID=" + b.id +
                        " X=" + b.x +
                        " Y=" + b.y +
                        " Width=" + b.width +
                        " Height=" + b.height);
                
                // check if tag is target
                if (b.id == 1) {
                    tagFound = true;
                    telemetry.addData("Locked Tag", "ID=" + b.id + " Center=(" + b.x + "," + b.y + ")");
                }*/
                switch(b.id)
                {
                    case 2:
                        sequenza.add(true);
                        sequenza.add(false);
                        sequenza.add(false);
                        break;
                        case 1:
                            sequenza.add(false);
                            sequenza.add(true);
                            sequenza.add(false);
                    break;
                    case 3:
                        sequenza.add(false);
                        sequenza.add(false);
                        sequenza.add(true);
                        break;
                }
                
                debug();
            }

            // if tag not found, then not visible
            if (!tagFound) {
                telemetry.addData("Locked Tag", "Not visible");
            }

            telemetry.update();
        }
    }

    public void debug()
    {
        for (int i = 0; i < 3; ++i)
        {
            for (boolean ball : sequenza) {
                telemetry.addData("Current Ball", ball ? "Verde" : "Viola");
            }
        }

        sequenza.clear();
    }
}