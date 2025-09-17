package org.firstinspires.ftc.team2844.Team2844_Decode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares.RobotHardware;

public class PrototypeClassify extends LinearOpMode {
    LinearOpMode opMode_;
    private ElapsedTime runtime = new ElapsedTime();
    RobotHardware robotHardware;

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this);
        opMode_ = this;
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double[] array = robotHardware.getColors();
            if(array[0] > 0.01 && array[1] > 0.01 && array[2] > 0.01){
                if(Math.max(array[1], array[2]) == array[1])
                {

                } else if(Math.max(array[1], array[2]) == array[2]) {

                }

            }
        }
    }
}
