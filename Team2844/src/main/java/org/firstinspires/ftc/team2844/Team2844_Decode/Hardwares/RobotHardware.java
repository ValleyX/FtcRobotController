package org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
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
    public double lFDriveSpeed = 0.0;
    public double rFDriveSpeed = 0.0;
    public double lBDriveSpeed = 0.0;
    public double rBDriveSpeed = 0.0;

    public double lFAlignSpeed = 0.0;
    public double rFAlignSpeed = 0.0;
    public double lBAlignSpeed = 0.0;
    public double rBAlignSpeed = 0.0;



    public final double TURN_THRESH = 2.3;
    public final double SLOW_THRESH = 15;
    public final double PGAIN = 0.023;



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

        leftFrontDrive.setPower(leftFront);
        leftBackDrive.setPower(leftBack);
        rightFrontDrive.setPower(rightFront);
        rightBackDrive.setPower(rightBack);
    }

    public void addDrivePower(double leftFront, double leftBack, double rightBack, double rightFront){
        lFDriveSpeed = leftFront;
        lBDriveSpeed = leftBack;
        rFDriveSpeed = rightFront;
        rBDriveSpeed = rightBack;

        if(Math.abs(lFDriveSpeed) >= 1){
            lFDriveSpeed /= Math.abs(lFDriveSpeed);
        }
        if(Math.abs(lBDriveSpeed) >= 1){
            lBDriveSpeed /= Math.abs(lBDriveSpeed);
        }
        if(Math.abs(rFDriveSpeed) >= 1){
            rFDriveSpeed /= Math.abs(rFDriveSpeed);
        }
        if(Math.abs(rBDriveSpeed) >= 1){
            rBDriveSpeed /= Math.abs(rBDriveSpeed);
        }
        calculateDrive();
    }

    public void addAlignPower(double leftFront, double leftBack, double rightBack, double rightFront){
        lFAlignSpeed = leftFront;
        lBAlignSpeed = leftBack;
        rFAlignSpeed = rightFront;
        rBAlignSpeed = rightBack;

        if(Math.abs(lFAlignSpeed) >= 1){
            lFAlignSpeed /= Math.abs(lFAlignSpeed);
        }
        if(Math.abs(lBAlignSpeed) >= 1){
            lBAlignSpeed /= Math.abs(lBAlignSpeed);
        }
        if(Math.abs(rFAlignSpeed) >= 1){
            rFAlignSpeed /= Math.abs(rFAlignSpeed);
        }
        if(Math.abs(rBAlignSpeed) >= 1){
            rBAlignSpeed /= Math.abs(rBAlignSpeed);
        }

        calculateDrive();
    }

    public void calculateDrive(){
        double leftFront = lFAlignSpeed + lFDriveSpeed;
        double leftBack = lBAlignSpeed + lBDriveSpeed;
        double rightFront = rFAlignSpeed + rFDriveSpeed;
        double rightBack = rBAlignSpeed + rBDriveSpeed;

        if(Math.abs(leftFront) >= 1){
            leftFront /= Math.abs(leftFront);
        }
        if(Math.abs(leftBack) >= 1){
            leftBack /= Math.abs(leftBack);
        }
        if(Math.abs(rightFront) >= 1){
            rightFront /= Math.abs(rightFront);
        }
        if(Math.abs(rightBack) >= 1){
            rightBack /= Math.abs(rightBack);
        }

        powerMotors(leftFront, leftBack, rightBack, rightFront);
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
        turnToFree(robotHeadingAngles()-llx, .6);
    }

    public void align(double llx){
        //Use the Limelight llx to fine tune
        turnTo(robotHeadingAngles()-llx, .4);
    }

    public void turnToEstimate(boolean red){
        //use the IMU to go to the estimated degrees
        double degrees = 0;
        if(red){
            //if we are red side and IMU is set to the back, turn sorta towards the goal
            degrees = -45;
        } else {
            degrees = 45;
         }

        turnToFree(degrees, 0.5);

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
        double heading = robotHeadingAngles();

        mult = Math.abs(Math.abs(degrees) - Math.abs(heading)) * PGAIN;
        if(!( degrees - TURN_THRESH < heading && heading < degrees + TURN_THRESH)) {
            if (degrees > 0) {
                oppDeg = degrees - 180;
                if (oppDeg < heading && heading < degrees) {
                    addAlignPower(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1), Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                } else {
                    addAlignPower(Math.min(speed * mult, 1), Math.min(speed * mult, 1), Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                }
            } else {
                oppDeg = degrees + 180;
                if (degrees < heading && heading < oppDeg) {
                    addAlignPower(Math.min(speed * mult, 1), Math.min(speed * mult, 1), Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                } else {
                    addAlignPower(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1), Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                }
            }
        } else {
            addAlignPower(0,0,0,0);
        }
    }
}
