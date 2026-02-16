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

        pattern = 0;
    }

    public SensorSubsystem(Limelight3A limelight, int pipelineNumber){
        pinpoint = null;

        this.limelight = limelight;

        //switches the pipeline
        limelight.pipelineSwitch(pipelineNumber);
        limelight.start();
        updateResult();

        pattern = 0;
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
        if(pinpoint.getPosition() != null){
            return pinpoint.getPosition();
        }
        return null;
    }

    public double getBotX(){
        if(pinpoint.getPosition() != null){
            return pinpoint.getPosX(DistanceUnit.INCH);
        }
        return Constants.NO_PP;
    }

    public double getBotY(){
        if(pinpoint.getPosition() != null){
            return pinpoint.getPosY(DistanceUnit.INCH);
        }
        return Constants.NO_PP;
    }


    public void setPinpointPose(Pose2D pose){
        pinpoint.setPosition(pose);
    }

    public double getHeadingFlipped(){
        double heading = getRobotHeading();

        if(heading >= 0.0){
            heading = (heading - 180);
        } else if (heading < 0.0){
            heading = (heading + 180);
        }

        return heading;
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
        if(pinpoint != null) {
            updateOrientation();
        }
        llResult = limelight.getLatestResult();
    }

    public double getTx(){
        if (llResult != null && llResult.isValid()) {
            return llResult.getTx();
        }
        return Constants.NO_LL;
    }

    public double hoodLinReg(){
        double distance = avgDis();
        if ((llResult != null && llResult.isValid()) && distance != Constants.NO_LL){
            return 0.0;
        } else {
            return 0.0;
        }
    }

    public double getDis(){
        if (llResult != null && llResult.isValid()){
            //Convert meters to inches
            return llResult.getBotposeAvgDist()*39.37008;
        }
        return Constants.NO_LL;
    }

    public double velocityLinReg(){
        double distance = avgDis();
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

    public void setPoseWithLL(){
        if(llResult != null && llResult.isValid()){
            setPinpointPose(new Pose2D(DistanceUnit.METER, getBotXLL(), getBotYLL(), AngleUnit.DEGREES, getRobotHeading()));
        }
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
        if(pinpoint != null) {
            setPoseWithLL();
            pinpoint.update();
        }

        if(getPattern() == 0){
            lookForPattern();
        }

        if(llResult.isValid() && llResult != null){
            setPoseWithLL();
        }
    }


    /* ---------- BOTH ---------- */

    public double pinpointDistance(){
        int pipeline = getPipeline();
        if(pipeline == 0 || pipeline == 2) {
            return Math.sqrt(Math.pow((Constants.BLUE_APRILTAG_X - getBotX()), 2) + Math.pow((Constants.BLUE_APRILTAG_Y - getBotY()), 2));
        } else if (pipeline == 1 || pipeline == 3) {
            return Math.sqrt(Math.pow((Constants.RED_APRILTAG_X - getBotX()), 2) + Math.pow((Constants.RED_APRILTAG_Y - getBotY()), 2));
        } else {
            return 0.0;
        }
    }

    public double avgDis(){
        double llDis = getDis();
        double pinpointDis = pinpointDistance();

        if(llDis != -999 && pinpoint != null){
            return (llDis+pinpointDis)/2;
        } else if (llDis != -999){
            return llDis;
        } else if (pinpoint != null){
            return pinpointDis;
        }

        return 0.0;
    }

    public double getPinpointTurretAngle(){
        int pipeline = getPipeline();
        double opposite = 0.0;
        double adjacent = 0.0;
        double hypotenuse = pinpointDistance();
        double angle = 0.0;

        double botX = getBotX();
        double botY = getBotY();
        double flippedHeading = -getHeadingFlipped();
        double heading = getRobotHeading();
        //double turretHeading = heading - (currentTurretAngle-90.0);

        double predictedAngle = 0.0;

        if(pipeline == 0 || pipeline == 2){
            opposite = Constants.BLUE_APRILTAG_X - getBotX();
            adjacent = Constants.BLUE_APRILTAG_Y - getBotY();

            angle = Math.toDegrees(Math.asin(opposite / hypotenuse));

            if(!Double.isNaN(angle)){
                double imuDisFromNinety;
                if(flippedHeading >= -90.0) {
                    imuDisFromNinety = (90.0 - flippedHeading);
                } else {
                    imuDisFromNinety = ((flippedHeading%90.0) - 90.0);
                }
                predictedAngle = imuDisFromNinety + angle;

                if (predictedAngle < 0.0) {
                    predictedAngle += 360;
                } else if (predictedAngle > 360) {
                    predictedAngle -= 360;
                }
            }

        } else if(pipeline == 1 || pipeline == 3) {
            opposite = Constants.RED_APRILTAG_X - getBotX();
            adjacent = Constants.RED_APRILTAG_Y - getBotY();

            angle = Math.toDegrees(Math.asin(opposite / hypotenuse));

            if(!Double.isNaN(angle)){
                double imuDisFromNinety;
                if(heading >= -90.0) {
                    imuDisFromNinety = (90.0 - heading);
                } else {
                    imuDisFromNinety = ((heading%90.0) - 90.0);
                }
                predictedAngle = imuDisFromNinety + angle;

                if (predictedAngle < 0.0) {
                    predictedAngle += 360;
                } else if (predictedAngle > 360) {
                    predictedAngle -= 360;
                }
            }
        }

        return predictedAngle;
    }



    public double getPinpointTurretAngleAuto(double botX, double botY, double heading){
        int pipeline = getPipeline();
        double opposite = 0.0;
        double adjacent = 0.0;
        double hypotenuse = pinpointDistanceAuto(botX, botY);
        double angle = 0.0;

        double flippedHeading = -flipHeading(heading);


        double predictedAngle = 0.0;

        if(pipeline == 0 || pipeline == 2){
            opposite = Constants.BLUE_APRILTAG_X - botX;
            adjacent = Constants.BLUE_APRILTAG_Y - botY;

            angle = Math.toDegrees(Math.asin(opposite / hypotenuse));

            if(!Double.isNaN(angle)){
                predictedAngle = flippedHeading + angle;

                if (predictedAngle < 0.0) {
                    predictedAngle += 360;
                } else if (predictedAngle > 360) {
                    predictedAngle -= 360;
                }
            }

        } else if(pipeline == 1 || pipeline == 3) {
            opposite = Constants.RED_APRILTAG_X - botX;
            adjacent = Constants.RED_APRILTAG_Y - botY;

            angle = Math.toDegrees(Math.asin(opposite / hypotenuse));

            if(!Double.isNaN(angle)){
                predictedAngle = heading + angle;

                if (predictedAngle < 0.0) {
                    predictedAngle += 360;
                } else if (predictedAngle > 360) {
                    predictedAngle -= 360;
                }
            }
        }

        return predictedAngle;
    }


    public double pinpointDistanceAuto(double botX, double botY){
        int pipeline = getPipeline();
        if(pipeline == 0 || pipeline == 2) {
            return Math.sqrt(Math.pow((Constants.BLUE_APRILTAG_X - botX), 2) + Math.pow((Constants.BLUE_APRILTAG_Y - botY), 2));
        } else if (pipeline == 1 || pipeline == 3) {
            return Math.sqrt(Math.pow((Constants.RED_APRILTAG_X - botX), 2) + Math.pow((Constants.RED_APRILTAG_Y - botY), 2));
        } else {
            return 0.0;
        }
    }

    public double flipHeading(double currentHeading){
        double heading = currentHeading;

        if(heading >= 0.0){
            heading = (heading - 180);
        } else if (heading < 0.0){
            heading = (heading + 180);
        }

        return heading;
    }
}
