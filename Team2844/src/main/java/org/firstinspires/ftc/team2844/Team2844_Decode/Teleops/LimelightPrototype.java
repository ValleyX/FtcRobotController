package org.firstinspires.ftc.team2844.Team2844_Decode.Teleops;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares.LimelightHardware;
import org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares.RobotHardware;

public class LimelightPrototype extends LinearOpMode {
    LinearOpMode opMode_;
    private ElapsedTime runtime = new ElapsedTime();
    RobotHardware robotHardware;
    LimelightHardware limelightHardware;

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this);
        limelightHardware = new LimelightHardware(this, true);
        opMode_ = this;

        //limelight stuff
        limelightHardware.limelight.start(); // Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.


        waitForStart();
        runtime.reset();

        while(opModeIsActive()){
            LLResult llResult = limelightHardware.limelight.getLatestResult();

            if (llResult != null && llResult.isValid()) {
                Pose3D botPose = llResult.getBotpose();  //pull in MT1 data
                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Tarea", llResult.getTa());

            }
        }
    }
}
