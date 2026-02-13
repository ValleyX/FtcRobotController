package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.SavedVars;

public class SensorSubsystem extends SubsystemBase {
    private GoBildaPinpointDriver pinpoint;

    /* ----------- Declarations ----------- */
    /**The man, the myth, the legend, the limelight3A*/
    private Limelight3A limelight;
    private LLResult llResult;
    private int pattern;
    private double imuOffset;

    public SensorSubsystem(GoBildaPinpointDriver pinpoint, Limelight3A limelight, int pipelineNumber){
        this.pinpoint = pinpoint;
        pinpoint.setOffsets(Constants.X_OFFSET, Constants.Y_OFFSET, DistanceUnit.INCH);
        pinpoint.resetPosAndIMU();
        pinpoint.initialize();

        //initialize the limelight
        this.limelight = limelight;

        //switches the pipeline
        limelight.pipelineSwitch(pipelineNumber);
        limelight.start();
        updateResult();
    }


    public double getRobotHeading(){
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }

    public double getRobotHeadingRadians(){
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }

    public void resetIMU(){
        pinpoint.recalibrateIMU();
    }


    public Pose2D getBotPose(){
        pinpoint.update();
        if(pinpoint.getPosition() != null){
            return pinpoint.getPosition();
        }
        return null;
    }

    public double getBotX(){
        pinpoint.update();
        if(pinpoint.getPosition() != null){
            return pinpoint.getPosX(DistanceUnit.INCH);
        }
        return Constants.NO_PP;
    }

    public double getBotY(){
        pinpoint.update();
        if(pinpoint.getPosition() != null){
            return pinpoint.getPosY(DistanceUnit.INCH);
        }
        return Constants.NO_PP;
    }


    public void setPinpointPose(Pose2D pose){
        pinpoint.setPosition(pose);
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
        updateOrientation();
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

    public void updateOrientation(){
        limelight.updateRobotOrientation(getRobotHeading());
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

    public void setPoseWithLL(){
        updateResult();
        if(llResult != null && llResult.isValid()){
            setPinpointPose(new Pose2D(DistanceUnit.METER, llResult.getBotpose().getPosition().x, llResult.getBotpose().getPosition().y, AngleUnit.DEGREES, getRobotHeading()));
        }
    }


    public int getPattern(){
        return pattern;
    }

    public void periodic(){
        updateResult();
        setPoseWithLL();
        pinpoint.update();
    }
}
