package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;

public class LimelightSubsystem extends SubsystemBase {
    Limelight3A limelight;
    int pattern;
    LLResult llResult;

    public LimelightSubsystem(HardwareMap hardwareMap, int pipeline){
        this.limelight = hardwareMap.get(Limelight3A.class, Constants.LL);
        pattern = 0;

        //Must be run for limelight to work
        limelight.pipelineSwitch(pipeline);
        limelight.start();
        updateResult();
    }

    /* -------------------- LIMELIGHT -------------------- */



    /*
     * This is the limelight Constructor
     * also requires the pipeline number to be passed in,
     * 0 is blue,
     * 1 is red
     * @param limelight Requires passing in a limelight object to set the limelight to
     * @param pipelineNumber The number of pipeline to use (0 is blue, 1 is red)
     */


    public int getPipeline(){
        return limelight.getStatus().getPipelineIndex();
    }
    public LLResult getLatestResult(){
        return limelight.getLatestResult();
    }

    public void updateResult(){
        llResult = limelight.getLatestResult();
    }

    public double getTx(){
        updateResult();
        if (llResult != null && llResult.isValid()) {
            return llResult.getTx();
        }
        return Constants.NO_LL;
    }

    public double hoodLinReg(){
        updateResult();
        double distance = getDis();
        if ((llResult != null && llResult.isValid()) && distance != Constants.NO_LL){
            return 0.0;
        } else {
            return 0.0;
        }
    }

    public double getDis(){
        updateResult();
        if (llResult != null && llResult.isValid()){
            //Convert meters to inches
            return llResult.getBotposeAvgDist()*39.37008;
        }
        return Constants.NO_LL;
    }

    public double velocityLinReg(){
        updateResult();
        double distance = getDis();
        if ((llResult != null && llResult.isValid()) && distance != Constants.NO_LL){
            return 0.0;
        } else {
            return 1500.0;
        }
    }

    public void updateOrientation(double heading){
        limelight.updateRobotOrientation(heading);
    }

    public Pose3D getBotPoseLL(){
        if (llResult != null && llResult.isValid()) {
            Pose3D botpose_mt2 = llResult.getBotpose_MT2();
            if (botpose_mt2 != null) {
                return botpose_mt2;
            }
        }
        return null;
    }

    public double getBotXLL(){
        updateResult();
        if(llResult != null && llResult.isValid()){
            return llResult.getBotpose().getPosition().x;
        }
        return Constants.NO_LL;
    }

    public double getBotYLL(){
        updateResult();
        if(llResult != null && llResult.isValid()){
            return llResult.getBotpose().getPosition().y;
        }
        return Constants.NO_LL;
    }


    public int getPattern(){
        return pattern;
    }

    public void periodic() {
        updateResult();
    }
}
