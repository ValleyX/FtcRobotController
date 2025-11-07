package org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotHardware {
    LinearOpMode opMode_;
    private final ElapsedTime runtime = new ElapsedTime();


    //drive motors
    public DcMotor rightBackDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftFrontDrive;
    //sensor and IMU
    public IMU imu;

    public final double TURN_THRESH = 1.5;


    public RobotHardware(LinearOpMode opMode) {
        opMode_ = opMode;

        //drive motor hardwarmaps
        rightFrontDrive = opMode_.hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = opMode_.hardwareMap.get(DcMotor.class, "rightBack");
        leftFrontDrive = opMode_.hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = opMode_.hardwareMap.get(DcMotor.class, "leftBack");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //IMU
        imu = opMode_.hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                        )
                )
        );

    }

    public void powerMotors(double leftFront, double leftBack, double rightBack, double rightFront){
        leftFrontDrive.setPower(leftFront);
        leftBackDrive.setPower(leftBack);
        rightFrontDrive.setPower(rightFront);
        rightBackDrive.setPower(rightBack);
    }


    public double robotHeadingRadians(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double robotHeadingAngles(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetImu(){
        imu.resetYaw();
    }


    public void alignFree(double llx){
        //Use the Limelight llx to fine tune without stoping the driver
        turnToFree(robotHeadingAngles()+llx, .4);
    }

    public void align(double llx){
        //Use the Limelight llx to fine tune
        turnTo(robotHeadingAngles()+llx, .4);
    }

    public void turnToEstimate(boolean red){
        //use the IMU to go to the estimated degrees
        if(red){
            //if we are red side and IMU is set to the back, turn sorta towards the goal
            turnTo(45, .8);
        } else {
            turnTo(-45, .8);
        }

    }

    public void turnTo(double degrees, double speed){
        double oppDeg;

        if(degrees <0){
            oppDeg = degrees + 180;
            if(oppDeg < robotHeadingAngles() && robotHeadingAngles() < degrees){
                while(!( degrees - TURN_THRESH < robotHeadingAngles() && robotHeadingAngles() < degrees + TURN_THRESH)){
                    powerMotors(-speed, -speed, speed, speed);
                }
            } else {
                while(!( degrees - TURN_THRESH < robotHeadingAngles() && robotHeadingAngles() < degrees + TURN_THRESH)){
                    powerMotors(speed, speed, -speed, -speed);
                }
            }
        } else {
            oppDeg = degrees - 180;
            if(degrees < robotHeadingAngles() && robotHeadingAngles() < oppDeg){
                while(!( degrees - TURN_THRESH < robotHeadingAngles() && robotHeadingAngles() < degrees + TURN_THRESH)){
                    powerMotors(speed, speed, -speed, -speed);
                }
            } else {
                while(!( degrees - TURN_THRESH < robotHeadingAngles() && robotHeadingAngles() < degrees + TURN_THRESH)){
                    powerMotors(-speed, -speed, speed, speed);
                }
            }
        }
    }

    public void turnToFree(double degrees, double speed){
        double oppDeg;

        if(degrees <0){
            oppDeg = degrees + 180;
            if(oppDeg < robotHeadingAngles() && robotHeadingAngles() < degrees){
                powerMotors(-speed, -speed, speed, speed);
            } else {
                powerMotors(speed, speed, -speed, -speed);
            }
        } else {
            oppDeg = degrees - 180;
            if(degrees < robotHeadingAngles() && robotHeadingAngles() < oppDeg){
                powerMotors(speed, speed, -speed, -speed);
            } else {
                powerMotors(-speed, -speed, speed, speed);
            }
        }
    }
}
