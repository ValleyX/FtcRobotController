package org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightHardware {
    private LinearOpMode opMode_;

    //limelight
    public Limelight3A limelight;
    LLResult llResult;

    public LimelightHardware(LinearOpMode opMode, boolean red){
        opMode_ = opMode;

        //hardware map
        limelight = opMode_.hardwareMap.get(Limelight3A.class, "limelight");

        //switch the pipeline
        if(red) {
            limelight.pipelineSwitch(0);
        } else {
            limelight.pipelineSwitch(1);
        }




        limelight.start();
        llResult = limelight.getLatestResult();
    }

    public LLResult getLatestResult(){
        return limelight.getLatestResult();
    }

    public void updateResult(){
        llResult = limelight.getLatestResult();
    }

    //Returns the y of the bot on the field through metatags
    public double getBotY(){
        updateResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();
            return botPose.getPosition().y;
        }
        return -999;
    }


    //Returns the X of the bot on the field through metatags
    public double getBotX(){
        updateResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();
            return botPose.getPosition().x;
        }
        return -999;
    }

    //Returns the rotation of the bot on the field through metatags
    public double getBotRot(){
        updateResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();
            return botPose.getPosition().z;
        }
        return -999;
    }

    public double getTy(){
        updateResult();
        if (llResult != null && llResult.isValid()) {
            return llResult.getTy();
        }
        return -999;
    }

    public double getTx(){
        updateResult();
        if (llResult != null && llResult.isValid()) {
            return llResult.getTx();
        }
        return -999;
    }

    public double getTarea(){
        updateResult();
        if (llResult != null && llResult.isValid()) {
            return llResult.getTa();
        }
        return -999;
    }
}
