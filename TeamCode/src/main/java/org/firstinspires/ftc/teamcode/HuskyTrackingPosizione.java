package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.LinkedList;
import java.util.List;

@TeleOp(name = "HuskyTrackingPosizione", group = "Sensor")
public class HuskyTrackingPosizione extends LinearOpMode {

    private HuskyLens huskyLens;
    private ElapsedTime runtime = new ElapsedTime();
    public List<Boolean> sequenza = new LinkedList<>();
    //VERDE == TRUE VIOLA == FALSE

    public boolean redAlliance = true;

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
            boolean ritorno = false;

            for (int i = 0; i < blocks.length; i++) {
                HuskyLens.Block b = blocks[i];

                if (redAlliance && b.id != 1)  continue;
                if (!redAlliance && b.id != 2)  continue;

                ritorno = VerificaPosizione(b);
                debug(ritorno);
            }

            // if tag not found, then not visible
            if (!tagFound) {
                telemetry.addData("Locked Tag", "Not visible");
            }

            telemetry.update();
        }
    }

    public boolean VerificaPosizione(HuskyLens.Block tag)
    {
            int error = tag.x - 160;
            float vToll = Math.abs(error);

            telemetry.addData("vToll value", vToll);

            if (vToll > 10) {
                if(error > 0)
                {
                    telemetry.addData("Predicted move", "Left");
                }
                else
                {
                    telemetry.addData("Predicted move", "Right");
                }
                return false;
            } else {
                return true;
            }
    }

    public void debug(boolean ritorno)
    {
        for (int i = 0; i < 3; ++i)
        {
            for (boolean ball : sequenza) {
                telemetry.addData("Current Ball", ball ? "Verde" : "Viola");
            }
        }
        if(ritorno) {
            telemetry.addData("Status", "Ok pos corretta");
        }
        else{
            telemetry.addData("Status", "Pos errata");
        }

        sequenza.clear();
    }
}