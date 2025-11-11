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


    //motor speed adding
    public double leftFrontSpeed = 0.0;
    public double rightFrontSpeed = 0.0;
    public double leftBackSpeed = 0.0;
    public double rightBackSpeed = 0.0;


    public final double TURN_THRESH = 2.3;
    public final double SLOW_THRESH = 15;
    public final double PGAIN = 0.15;



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
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );

    }

    public void powerMotors(double leftFront, double leftBack, double rightBack, double rightFront){
        leftFrontSpeed = leftFront;
        rightFrontSpeed = rightFront;
        leftBackSpeed = leftBack;
        rightBackSpeed = rightBack;

        leftFrontDrive.setPower(leftFrontSpeed);
        leftBackDrive.setPower(leftBackSpeed);
        rightFrontDrive.setPower(rightFrontSpeed);
        rightBackDrive.setPower(rightBackSpeed);
    }

    public void addDrivePower(double leftFront, double leftBack, double rightBack, double rightFront){
        leftFrontSpeed += leftFront;
        leftBackSpeed += leftBack;
        rightFrontSpeed += rightFront;
        rightBackSpeed += rightBack;

        if(Math.abs(leftFrontSpeed) == 1){
            leftFrontSpeed /= Math.abs(leftFrontSpeed);
        }
        if(Math.abs(leftBackSpeed) == 1){
            leftBackSpeed /= Math.abs(leftBackSpeed);
        }
        if(Math.abs(rightFrontSpeed) == 1){
            rightFrontSpeed /= Math.abs(rightFrontSpeed);
        }
        if(Math.abs(rightBackSpeed) == 1){
            rightBackSpeed /= Math.abs(rightBackSpeed);
        }
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
        turnToFree(robotHeadingAngles()-llx, .4);
    }

    public void align(double llx){
        //Use the Limelight llx to fine tune
        turnTo(robotHeadingAngles()-llx, .4);
    }

    public void turnToEstimate(boolean red){
        //use the IMU to go to the estimated degrees
        if(red){
            //if we are red side and IMU is set to the back, turn sorta towards the goal
            turnTo(-45, .5);
        } else {
            turnTo(45, .5);
        }

    }

    public void turnTo(double degrees, double speed){
        double oppDeg;
        double mult;
        mult = Math.abs(Math.abs(degrees) + robotHeadingAngles()) * PGAIN;
        if(degrees >0){
            oppDeg = degrees - 180;
            if(oppDeg < robotHeadingAngles() && robotHeadingAngles() < degrees){
                while(!( degrees + TURN_THRESH <= robotHeadingAngles() && robotHeadingAngles() <= degrees - TURN_THRESH) && opMode_.opModeIsActive()){
                    System.out.println("2844 - Left side Positive");
                    mult = Math.abs(Math.abs(degrees) - robotHeadingAngles()) * PGAIN;
                    System.out.println("2844 - Error: " + (degrees-robotHeadingAngles()));
                    if((robotHeadingAngles() < degrees )) {
                        System.out.println("2844 - turning left");
                        powerMotors(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1), Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                    }else{
                        System.out.println("2844 - turning right");
                        powerMotors(Math.min(speed * mult, 1), Math.min(speed * mult, 1), Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                    }
                    System.out.println("2844 - Mult: " + mult);
                }
            } else {
                while(!( degrees + TURN_THRESH <= robotHeadingAngles() && robotHeadingAngles() <= degrees - TURN_THRESH) && opMode_.opModeIsActive()){
                    System.out.println("2844 - Right Side, Positive");
                    mult = Math.abs(Math.abs(degrees) - robotHeadingAngles()) * PGAIN;
                    System.out.println("2844 - Error: " + (degrees-robotHeadingAngles()));
                    if((robotHeadingAngles() > degrees)) {
                        System.out.println("2844 - turning right");
                        powerMotors(Math.min(speed * mult, 1), Math.min(speed * mult, 1), Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                    }else{
                        System.out.println("2844 - Turning left");
                        powerMotors(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1), Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                    }
                    System.out.println("2844 - Mult: " + mult);
                }
            }
        } else {
            oppDeg = degrees + 180;
            if(degrees < robotHeadingAngles() && robotHeadingAngles() < oppDeg){
                while(!( degrees - TURN_THRESH < robotHeadingAngles() && robotHeadingAngles() < degrees + TURN_THRESH) && opMode_.opModeIsActive()){
                    System.out.println("2844 - Left side, Negative");
                    mult = Math.abs(Math.abs(degrees) + robotHeadingAngles()) * PGAIN;
                    System.out.println("2844 - Error: " + (degrees-robotHeadingAngles()));
                    if((robotHeadingAngles() < degrees)) {
                        System.out.println("2844 - turning right");
                        powerMotors(Math.min(speed * mult, 1), Math.min(speed * mult, 1), Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                    }else{
                        System.out.println("2844 - turning left");
                        powerMotors(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1), Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                    }
                    System.out.println("2844 - Mult: " + mult);
                }
            } else {
                while(!( degrees - TURN_THRESH < robotHeadingAngles() && robotHeadingAngles() < degrees + TURN_THRESH) && opMode_.opModeIsActive()){
                    System.out.println("2844 - right side, Negative");
                    mult = Math.abs(Math.abs(degrees) + robotHeadingAngles()) * PGAIN;
                    System.out.println("2844 - Error: " + (degrees-robotHeadingAngles()));
                    if((robotHeadingAngles() < degrees)) {
                        System.out.println("2844 - turning right");
                        powerMotors(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1), Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                    }else{
                        System.out.println("2844 - Turning left");
                        powerMotors(Math.min(speed * mult, 1), Math.min(speed * mult, 1), Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                    }
                    System.out.println("2844 - Mult: " + mult);
                }
            }
        }
    }

    public void turnToFree(double degrees, double speed){
        double oppDeg;
        double mult;

        mult = Math.abs(Math.abs(degrees) - Math.abs(robotHeadingAngles())) * PGAIN;
        if(degrees >0){
            oppDeg = degrees - 180;
            if(oppDeg < robotHeadingAngles() && robotHeadingAngles() < degrees){
                if(!( degrees - TURN_THRESH < robotHeadingAngles() && robotHeadingAngles() < degrees + TURN_THRESH)) {
                    if ((robotHeadingAngles() < degrees))
                        powerMotors(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1), Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                    else{
                        powerMotors(Math.min(speed * mult, 1), Math.min(speed * mult, 1), Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                    }
                }
            } else {
                if(!( degrees - TURN_THRESH < robotHeadingAngles() && robotHeadingAngles() < degrees + TURN_THRESH)) {
                    if ((robotHeadingAngles() > degrees))
                        powerMotors(Math.min(speed * mult, 1), Math.min(speed * mult, 1), Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                    else{
                        powerMotors(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1), Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                    }
                }
            }
        } else {
            oppDeg = degrees + 180;
            if(degrees < robotHeadingAngles() && robotHeadingAngles() < oppDeg){
                if(!( degrees - TURN_THRESH < robotHeadingAngles() && robotHeadingAngles() < degrees + TURN_THRESH)) {
                    if ((robotHeadingAngles() > degrees))
                        powerMotors(Math.min(speed * mult, 1), Math.min(speed * mult, 1), Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                    else{
                        powerMotors(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1), Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                    }
                }
            } else {
                if(!(degrees - TURN_THRESH < robotHeadingAngles() && robotHeadingAngles() < degrees + TURN_THRESH)) {
                    if ((robotHeadingAngles() < degrees))
                        powerMotors(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1), Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                    else{
                        powerMotors(Math.min(speed * mult, 1), Math.min(speed * mult, 1), Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                    }
                }
            }
        }
    }
}
