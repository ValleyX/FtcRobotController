package org.firstinspires.ftc.team2844.Team2844_Decode.Teleops;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares.RobotHardware;

public class LimelightPrototype extends LinearOpMode {
    LinearOpMode opMode_;
    private ElapsedTime runtime = new ElapsedTime();
    RobotHardware robotHardware;

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this);
        opMode_ = this;

        //limelight stuff
        robotHardware.limelight.pipelineSwitch(9);
        robotHardware.limelight.start(); // Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.


        waitForStart();
        runtime.reset();

        while(opModeIsActive()){
            LLResult result = robotHardware.limelight.getLatestResult();

        }
    }
}
