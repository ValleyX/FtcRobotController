package org.firstinspires.ftc.team12841.teleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Practice")
public class TeleOpPractice extends TeleOpBase {

    @Override
    public void runOpMode() throws InterruptedException {
        // Set the pipeline before the base class initializes hardware
        super.pipeline = 0;

        // Run the main TeleOp loop
        super.runOpMode();
    }
}