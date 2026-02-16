package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.SavedVars;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Roadrunner.MecanumDrive;

//import org.firstinspires.ftc.team2844.Team2844_Decode.RoadrunnerStuff.RoadrunnerQuickstart.MecanumDrive;

public class DriveSubsystem extends SubsystemBase {

    private MecanumDrive drive;

    public DriveSubsystem(HardwareMap hardwareMap){
        drive = new MecanumDrive(hardwareMap, new Pose2d(SavedVars.startingX, SavedVars.startingY, SavedVars.startingHeading));
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed, double heading) {
        double botHeading = heading;
        //botHeading = 0;

        //code for field centric (Idk how it works, pretty sure it's magic or makes triangles or something)
        //REMEMBER IT USES RADIANS
        double rotX = strafeSpeed * Math.cos(-botHeading) - forwardSpeed * Math.sin(-botHeading);
        double rotY = strafeSpeed * Math.sin(-botHeading) + forwardSpeed * Math.cos(-botHeading);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turnSpeed), 1);
        double frontLeftPower = (rotY + (rotX * Constants.STRAFE_CORRECTION) + turnSpeed) / denominator;
        double backLeftPower = (rotY - rotX + turnSpeed) / denominator;
        double frontRightPower = (rotY - (rotX * Constants.STRAFE_CORRECTION) - turnSpeed) / denominator;
        double backRightPower = (rotY + rotX - turnSpeed) / denominator;

        drive.leftFront.setPower(frontLeftPower);
        drive.rightFront.setPower(frontRightPower);
        drive.leftBack.setPower(backLeftPower);
        drive.rightBack.setPower(backRightPower);


    }

    public double getRobotHeading(){
        return drive.getRobotHeading();
    }

    public double getRobotHeadingRadians(){
        return Math.toRadians(drive.getRobotHeading());
    }

    public void resetIMU(){
        drive.localizer.setPose(new Pose2d(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, 0.0));
    }


    public Pose2d getBotPose(){
        if(drive.localizer.getPose() != null){
            return drive.localizer.getPose();
        }
        return null;
    }

    public double getBotX(){
        return drive.getRobotX();
    }

    public double getBotY(){
        return drive.getRobotY();
    }


    public void setPinpointPose(Pose2d pose){
        drive.localizer.setPose(pose);
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


    public double pinpointDistance(int pipeline){
        if(pipeline == 0 || pipeline == 2) {
            return Math.sqrt(Math.pow((Constants.BLUE_APRILTAG_X - getBotX()), 2) + Math.pow((Constants.BLUE_APRILTAG_Y - getBotY()), 2));
        } else if (pipeline == 1 || pipeline == 3) {
            return Math.sqrt(Math.pow((Constants.RED_APRILTAG_X - getBotX()), 2) + Math.pow((Constants.RED_APRILTAG_Y - getBotY()), 2));
        } else {
            return 0.0;
        }
    }


    public double getPinpointTurretAngle(int pipeline){
        double opposite = 0.0;
        double adjacent = 0.0;
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

            angle = Math.toDegrees(Math.atan2(adjacent, opposite));

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

            angle = Math.toDegrees(Math.atan2(adjacent, opposite));

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



    public double getPinpointTurretAngleAuto(double botX, double botY, double heading, int pipeline){
        double opposite = 0.0;
        double adjacent = 0.0;
        double hypotenuse = pinpointDistanceAuto(botX, botY, pipeline);
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


    public double pinpointDistanceAuto(double botX, double botY, int pipeline){
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


    public double hoodLinReg(int pipeline){
        double distance = pinpointDistance(pipeline);
        return distance;
    }



    public double velocityLinReg(int pipeline){
        double distance = pinpointDistance(pipeline);
        return 1500;
    }


    public void setPoseWithLL(Pose3D pose){
        if(pose != null){
            drive.localizer.setPose(new Pose2d(pose.getPosition().x,pose.getPosition().y, getRobotHeading()));
        }
    }

    /* this is a option for the subsystem
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }*/
}
