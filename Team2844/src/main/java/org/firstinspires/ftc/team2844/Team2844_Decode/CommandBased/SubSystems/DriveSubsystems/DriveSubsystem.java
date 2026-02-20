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

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

//import org.firstinspires.ftc.team2844.Team2844_Decode.RoadrunnerStuff.RoadrunnerQuickstart.MecanumDrive;

public class DriveSubsystem extends SubsystemBase {

    public MecanumDrive drive;

    public DriveSubsystem(HardwareMap hardwareMap){
        drive = new MecanumDrive(hardwareMap, new Pose2d(SavedVars.startingX, SavedVars.startingY, SavedVars.startingHeading));
        //drive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
    }

    public DriveSubsystem(HardwareMap hardwareMap, Pose2d pose){
        drive = new MecanumDrive(hardwareMap, pose);
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed, DoubleSupplier heading) {
        drive.updatePoseEstimate();
        double botHeading = -heading.getAsDouble();
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
        return Math.toDegrees(drive.localizer.getPose().heading.toDouble());
    }

    public double getRobotHeadingRadians(){
        return (drive.localizer.getPose().heading.toDouble());
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
        return drive.localizer.getPose().position.x;
    }

    public double getBotY(){
        return drive.localizer.getPose().position.y;
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
        double botX = getBotX();
        double botY = getBotY();
        double angle = 0.0;
        double tempHeading = getRobotHeading();

        double limelightX = 0.0;
        double limelightY = 0.0;


        if(pipeline == Constants.BLUE_PIPELINE || pipeline == Constants.BLUE_PIPELINE_MOTIF){
            limelightX = Constants.BLUE_APRILTAG_X;
            limelightY = Constants.BLUE_APRILTAG_Y;
            tempHeading = tempHeading;
        } else if(pipeline == Constants.RED_PIPELINE || pipeline == Constants.RED_PIPELINE_MOTIF){
            limelightX = Constants.RED_APRILTAG_X;
            limelightY = Constants.RED_APRILTAG_Y;
            tempHeading = -tempHeading;
        }

        opposite = limelightX - botX;
        adjacent = limelightY - botY;

        angle = Math.toDegrees(Math.atan2(adjacent, opposite));

        double turretAngle = (360.0-angle)-tempHeading;

        if(turretAngle > 360.0){
            turretAngle -= 360.0;
        } else if (turretAngle < 0.0){
            turretAngle += 360.0;
        }

        return Math.max(Math.min(300, (360.0-turretAngle)-tempHeading), 10);
        //return 90.0;
    }



    public double getPinpointTurretAngleAuto(double botX, double botY, double heading, int pipeline){
        double opposite = 0.0;
        double adjacent = 0.0;
        double angle = 0.0;
        double tempHeading = heading;

        double limelightX = 0.0;
        double limelightY = 0.0;


        if(pipeline == Constants.BLUE_PIPELINE || pipeline == Constants.BLUE_PIPELINE_MOTIF){
            limelightX = Constants.BLUE_APRILTAG_X;
            limelightY = Constants.BLUE_APRILTAG_Y;
            tempHeading = heading;
        } else if(pipeline == Constants.RED_PIPELINE || pipeline == Constants.RED_PIPELINE_MOTIF){
            limelightX = Constants.RED_APRILTAG_X;
            limelightY = Constants.RED_APRILTAG_Y;
            tempHeading = -heading;
        }

        opposite = limelightX - botX;
        adjacent = limelightY - botY;

        angle = Math.toDegrees(Math.atan2(adjacent, opposite));

        return Math.max(Math.min(330, (360.0-angle)-tempHeading), 10);
        //return 90.0;
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
        return 0.0;
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
