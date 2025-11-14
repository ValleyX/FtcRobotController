package org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class LimelightHardware {
    private LinearOpMode opMode_;

    //limelight
    public Limelight3A limelight;
    LLResult llResult;
    boolean pipelineCalled = false;

    public LimelightHardware(LinearOpMode opMode){
        opMode_ = opMode;

        //hardware map
        limelight = opMode_.hardwareMap.get(Limelight3A.class, "limelight");
    }
    public void innit(int pipeline){
        limelight.pipelineSwitch(pipeline);
        limelight.start();
        pipelineCalled = true;
    }

    public int getPipeline(){
        return limelight.getStatus().getPipelineIndex();
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

    public double getBotCamZ(){
        updateResult();
        if (llResult != null && llResult.isValid()) {
            List< LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            return ((fiducials.get(0).getCameraPoseTargetSpace().getPosition().z *100)/2.54)*-1.635;
        }
        return -999;
    }

    public double getBotCamY(){
        updateResult();
        if (llResult != null && llResult.isValid()) {
            List< LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            return ((fiducials.get(0).getCameraPoseTargetSpace().getPosition().y *100)/2.54)*-1.635;
        }
        return -999;
    }

    public double getBotCamX(){
        updateResult();
        if (llResult != null && llResult.isValid()) {
            List< LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            return ((fiducials.get(0).getCameraPoseTargetSpace().getPosition().x *100)/2.54)*-1.635;
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
