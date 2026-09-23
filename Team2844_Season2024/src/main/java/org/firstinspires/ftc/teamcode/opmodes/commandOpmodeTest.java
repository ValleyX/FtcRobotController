package org.firstinspires.ftc.team12841.opmodes;



import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;


//framework for creating opmodes
@Disabled
@TeleOp(name = "test")
public class commandOpmodeTest extends CommandOpMode {


    @Override
    public void initialize() /*throws InterruptedException*/ {


    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();




        // run the scheduler
        //TODO; add telemetry into here if not already
        while (!isStopRequested() && opModeIsActive()) {
            run();

        }
        reset();
    }

}