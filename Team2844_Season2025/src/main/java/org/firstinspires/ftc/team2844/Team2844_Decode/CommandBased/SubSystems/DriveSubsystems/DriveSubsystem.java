package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.PedroConstants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.SavedVars;

import java.util.function.DoubleSupplier;

public class DriveSubsystem extends SubsystemBase {

    public MecanumDrive drive;
    public Follower follower;
    private double headingOffset;



    public DriveSubsystem(HardwareMap hardwareMap, int pipeline) {
        double temp = SavedVars.startingHeading;

        follower = PedroConstants.createFollower(hardwareMap);

        if (temp == Constants.NO_HEADING || temp == (Constants.NO_HEADING % 360.0)) {
            // No prior auto — use pipeline defaults for both RR init and FC offset
            if (pipeline == Constants.BLUE_PIPELINE) {
                drive = new MecanumDrive(hardwareMap, new Pose2d(SavedVars.startingX, SavedVars.startingY, Math.toRadians(-90.0)));
                headingOffset = Math.toRadians(-90);
            } else if (pipeline == Constants.RED_PIPELINE) {
                drive = new MecanumDrive(hardwareMap, new Pose2d(SavedVars.startingX, SavedVars.startingY, Math.toRadians(90.0)));
                headingOffset = Math.toRadians(90.0);
            } else {
                drive = new MecanumDrive(hardwareMap, new Pose2d(SavedVars.startingX, SavedVars.startingY, 0.0));
                headingOffset = 0.0;
            }
        } else {
            // Prior auto exists — temp is in degrees (RR convention), same convention as Pedro
            // headingOffset rotates FC reference to compensate for where the bot actually ended up
            drive = new MecanumDrive(hardwareMap, new Pose2d(SavedVars.startingX, SavedVars.startingY, Math.toRadians(temp)));
            headingOffset = -Math.toRadians(temp);
        }

        follower.startTeleOpDrive(true);
    }

    public DriveSubsystem(HardwareMap hardwareMap, Pose2d pose) {
        drive = new MecanumDrive(hardwareMap, pose);
    }

    public void teleopDrive(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        follower.setTeleOpDrive(strafeSpeed, forwardSpeed, turnSpeed, false, headingOffset);
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed, DoubleSupplier heading) {
        drive.updatePoseEstimate();
        double botHeading = heading.getAsDouble();
        //botHeading = 0;

        //code for field centric (Idk how it works, pretty sure it's magic or makes triangles or something)
        //REMEMBER IT USES RADIANS
        double rotX = strafeSpeed * Math.cos(botHeading) - forwardSpeed * Math.sin(botHeading);
        double rotY = strafeSpeed * Math.sin(botHeading) + forwardSpeed * Math.cos(botHeading);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turnSpeed), 1);
        double frontLeftPower = (rotY - (rotX) + turnSpeed) / denominator;
        double backLeftPower = (rotY + rotX + turnSpeed) / denominator;
        double frontRightPower = (rotY + (rotX) - turnSpeed) / denominator;
        double backRightPower = (rotY - rotX - turnSpeed) / denominator;

        frontLeftPower = Math.min(1, Math.max(frontLeftPower  * Constants.DRIVE_CORRECTION, -1));
        backLeftPower = Math.min(1, Math.max(backLeftPower  * Constants.DRIVE_CORRECTION, -1));
        frontRightPower = Math.min(1, Math.max(frontRightPower  * Constants.DRIVE_CORRECTION, -1));
        backRightPower = Math.min(1, Math.max(backRightPower  * Constants.DRIVE_CORRECTION, -1));

        drive.leftFront.setPower(frontLeftPower);
        drive.rightFront.setPower(frontRightPower);
        drive.leftBack.setPower(backLeftPower);
        drive.rightBack.setPower(backRightPower);
    }

    /*public void setDrivePower(double frontLeft, double backLeft, double frontRight, double backRight) {

        drive.leftFront.setPower(frontLeft);
        drive.rightFront.setPower(frontRight);
        drive.leftBack.setPower(backLeft);
        drive.rightBack.setPower(backRight);
    }*/


    public double getRobotHeading() {
        return Math.toDegrees(drive.localizer.getPose().heading.toDouble());
    }

    public double getRobotDriveHeading(int pipeline) {
        double heading = getRobotHeading();
        if (pipeline == Constants.BLUE_PIPELINE) heading -= 90;
        else if (pipeline == Constants.RED_PIPELINE) heading += 90;
        while (heading > 180) heading -= 360;
        while (heading <= -180) heading += 360;
        return Math.toRadians(heading);
    }

    public double getRobotBlueDriveHeading() {
        double heading = getRobotHeading() - 90;
        while (heading > 180) heading -= 360;
        while (heading <= -180) heading += 360;
        return Math.toRadians(heading);
    }

    public double getRobotRedDriveHeading() {
        double heading = getRobotHeading() + 90;
        while (heading > 180) heading -= 360;
        while (heading <= -180) heading += 360;
        return Math.toRadians(heading);
    }

    public double getRobotHeadingRadians() {
        return drive.localizer.getPose().heading.toDouble();
    }

    public void resetIMU() {
        //drive.localizer.setPose(new Pose2d(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, 0.0));

    }




    public void resetIMU(double degrees) {
        drive.localizer.setPose(new Pose2d(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, Math.toRadians(degrees)));
        follower.setPose(new Pose(drive.localizer.getPose().position.x,drive.localizer.getPose().position.y, Math.toRadians(degrees)));
    }

    public Pose2d getBotPose() {
        return drive.localizer.getPose();
    }

    public double getBotX() {
        return drive.localizer.getPose().position.x;
    }

    public double getBotY() {
        return drive.localizer.getPose().position.y;
    }

    public void setPinpointPose(Pose2d pose) {
        drive.localizer.setPose(pose);
    }

    public double getHeadingFlipped() {
        double heading = getRobotHeading();
        return heading >= 0.0 ? heading - 180 : heading + 180;
    }

    public double pinpointDistance(int pipeline) {
        if (pipeline == 0 || pipeline == 2) {
            return Math.sqrt(Math.pow(Constants.BLUE_APRILTAG_X - getBotX(), 2) + Math.pow(Constants.BLUE_APRILTAG_Y - getBotY(), 2));
        } else if (pipeline == 1 || pipeline == 3) {
            return Math.sqrt(Math.pow(Constants.RED_APRILTAG_X - getBotX(), 2) + Math.pow(Constants.RED_APRILTAG_Y - getBotY(), 2));
        }
        return 0.0;
    }

    public double getPinpointTurretAngle(int pipeline) {
        double botX = getBotX(), botY = getBotY();
        double tempHeading = getRobotHeading();
        double angle = 0.0;

        double shooterX = botX + Constants.RADIUS_FROM_CENTER * Math.cos(Math.toRadians(tempHeading));
        double shooterY = botY + Constants.RADIUS_FROM_CENTER * Math.sin(Math.toRadians(tempHeading));

        double limelightX, limelightY;
        if (pipeline == Constants.BLUE_PIPELINE || pipeline == Constants.BLUE_PIPELINE_MOTIF) {
            limelightX = Constants.BLUE_APRILTAG_X; limelightY = Constants.BLUE_APRILTAG_Y;
            angle = 270.0 - Math.toDegrees(Math.atan2(-distanceFormula(limelightX, limelightY, shooterX, limelightY), -distanceFormula(limelightX, limelightY, limelightX, shooterY)));
        } else if (pipeline == Constants.RED_PIPELINE || pipeline == Constants.RED_PIPELINE_MOTIF) {
            limelightX = Constants.RED_APRILTAG_X; limelightY = Constants.RED_APRILTAG_Y;
            angle = 90.0 + Math.toDegrees(Math.atan2(-distanceFormula(limelightX, limelightY, shooterX, limelightY), -distanceFormula(limelightX, limelightY, limelightX, shooterY)));
        }

        double turretAngle = angle - tempHeading;
        if (turretAngle > 360.0) turretAngle -= 360.0;
        else if (turretAngle < 0.0) turretAngle += 360.0;
        return Math.max(Math.min(Constants.MAX_DEGREE, turretAngle), Constants.MIN_DEGREE);
    }

    public double getPinpointTurretAngleAuto(double botX, double botY, double heading, int pipeline) {
        double limelightX = 0.0, limelightY = 0.0;
        if (pipeline == Constants.BLUE_PIPELINE || pipeline == Constants.BLUE_PIPELINE_MOTIF) {
            limelightX = -Constants.BLUE_APRILTAG_X; limelightY = Constants.BLUE_APRILTAG_Y;
        } else if (pipeline == Constants.RED_PIPELINE || pipeline == Constants.RED_PIPELINE_MOTIF) {
            limelightX = -Constants.RED_APRILTAG_X; limelightY = Constants.RED_APRILTAG_Y;
        }
        double angle = Math.toDegrees(Math.atan2(limelightX - botX, limelightY - botY));
        double turretAngle = (360.0 - angle) - heading;
        if (turretAngle > 360.0) turretAngle -= 360.0;
        else if (turretAngle < 0.0) turretAngle += 360.0;
        return Math.max(Math.min(Constants.MAX_DEGREE, turretAngle), Constants.MIN_DEGREE);
    }

    public double pinpointDistanceAuto(double botX, double botY, int pipeline) {
        if (pipeline == 0 || pipeline == 2) {
            return Math.sqrt(Math.pow(Constants.BLUE_APRILTAG_X - botX, 2) + Math.pow(Constants.BLUE_APRILTAG_Y - botY, 2));
        } else if (pipeline == 1 || pipeline == 3) {
            return Math.sqrt(Math.pow(Constants.RED_APRILTAG_X - botX, 2) + Math.pow(Constants.RED_APRILTAG_Y - botY, 2));
        }
        return 0.0;
    }

    public double flipHeading(double currentHeading) {
        return currentHeading >= 0.0 ? currentHeading - 180 : currentHeading + 180;
    }

    public double hoodLinReg(int pipeline) {
        return Math.max(Math.min((.0038895 * pinpointDistance(pipeline)) - 0.111817, 1.0), 0.0);
    }

    public double velocityLinReg(int pipeline) {
        double d = pinpointDistance(pipeline);
        return (0.00000921885 * Math.pow(d, 4)) - (0.00288935 * Math.pow(d, 3))
                + (0.317132 * Math.pow(d, 2)) - (9.88495 * d) + 993.83429;
    }

    public void setPoseWithLL(Pose3D pose) {
        if (pose != null) {
            drive.localizer.setPose(new Pose2d(pose.getPosition().x, pose.getPosition().y, getRobotHeading()));
        }
    }

    private double distanceFormula(double a, double b, double x, double y) {
        return Math.sqrt(Math.pow(a - x, 2) + Math.pow(b - y, 2));
    }

    @Override
    public void periodic() {
        drive.updatePoseEstimate();
        if (follower != null) follower.update();
    }
}