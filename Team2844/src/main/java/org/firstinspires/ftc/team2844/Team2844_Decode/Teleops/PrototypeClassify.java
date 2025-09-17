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
        opMode_ = new LinearOpMode(this, robotHardware);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if(){

            }
        }
    }
}
