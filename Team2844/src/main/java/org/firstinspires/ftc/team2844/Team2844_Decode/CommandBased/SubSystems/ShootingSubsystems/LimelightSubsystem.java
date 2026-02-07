package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Constants;

public class LimelightSubsystem extends SubsystemBase {

    /* ----------- Declarations ----------- */
    /**The man, the myth, the legend, the limelight3A*/
    private Limelight3A limelight;
    private LLResult llResult;

    /**
     * This is the limelight Constructor
     * also requires the pipeline number to be passed in,
     * 0 is blue,
     * 1 is red
     * @param limelight Requires passing in a limelight object to set the limelight to
     * @param pipelineNumber The number of pipeline to use (0 is blue, 1 is red)
     */
    public LimelightSubsystem(Limelight3A limelight, int pipelineNumber){
        //initialize the limelight
        this.limelight = limelight;

        //switches the pipeline
        limelight.pipelineSwitch(pipelineNumber);
        limelight.start();
        updateResult();
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
            return distance;
        }
        return Constants.NO_LL;
    }

    public double getDis(){
        updateResult();
        if (llResult != null && llResult.isValid()){
            return 0.0;
        }
        return Constants.NO_LL;
    }

    public double velocityLinReg(){
        updateResult();
        double distance = getDis();
        if ((llResult != null && llResult.isValid()) && distance != Constants.NO_LL){
            return distance;
        }
        return Constants.NO_LL;
    }
}
