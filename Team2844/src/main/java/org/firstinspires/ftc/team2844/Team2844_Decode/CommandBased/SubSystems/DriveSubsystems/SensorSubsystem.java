package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems;

import static java.lang.Double.NaN;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;

public class SensorSubsystem extends SubsystemBase {

    /* ----------- Declarations ----------- */
    /**The man, the myth, the legend, the limelight3A*/
    private Limelight3A limelight;
    private LLResult llResult;
    private int pattern;
    private double imuOffset;

    public SensorSubsystem(Limelight3A limelight, int pipelineNumber){
        this.limelight = limelight;

        //switches the pipeline
        limelight.pipelineSwitch(pipelineNumber);
        limelight.start();
        updateResult();

        pattern = 0;
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

    public void setPipeline(int pipeline){
        limelight.pipelineSwitch(pipeline);
    }
    public LLResult getLatestResult(){
        return limelight.getLatestResult();
    }

    public void updateResult(){
        llResult = limelight.getLatestResult();
    }
//this returns the offset  of the crossharis to the center of the target
    //thi sis typically -27 to 27 deg
    public double getTx(){
        if (llResult != null && llResult.isValid()) {
            return llResult.getTx();
        }
        return Constants.NO_LL;
    }

    public void updateOrientation(double heading){
        limelight.updateRobotOrientation(heading);
    }

    public double getDis(){
        if (llResult != null && llResult.isValid()){
            //Convert meters to inches
            return llResult.getBotposeAvgDist()*39.37008;
        }
        return Constants.NO_LL;
    }

    public Pose3D getBotPoseLL(){
        if (llResult != null && llResult.isValid()) {
            Pose3D botpose = llResult.getBotpose();
            if (botpose != null) {
                return botpose;
            }
        }
        return null;
    }

    public double getBotXLL(){
        if(llResult != null && llResult.isValid()){
            return getBotPoseLL().getPosition().x;
        }
        return Constants.NO_LL;
    }

    public double getBotYLL(){
        if(llResult != null && llResult.isValid()){
            return getBotPoseLL().getPosition().y;
        }
        return Constants.NO_LL;
    }


    public int getPattern(){
        return pattern;
    }

    public void setPattern(int pattern){
        this.pattern = pattern;
    }

    public void lookForPattern(){
        if(llResult != null && llResult.isValid() && pattern == 0){
            int id = llResult.getFiducialResults().get(0).getFiducialId();

            if(id== 21){
                pattern = Constants.PATTERN_GPP;
            } else if (id == 22){
                pattern = Constants.PATTERN_PGP;
            } else if (id == 23){
                pattern = Constants.PATTERN_PPG;
            }
        }
    }

    public void periodic(){
        updateResult();

        if(getPattern() == 0){
            lookForPattern();
        }
    }


    /* ---------- BOTH ---------- */


    public double avgDis(double pinpointDis){
        double llDis = getDis();

        if(llDis != Constants.NO_LL) {
            return (llDis + pinpointDis) / 2;
        }
        return 0.0;
    }
}
