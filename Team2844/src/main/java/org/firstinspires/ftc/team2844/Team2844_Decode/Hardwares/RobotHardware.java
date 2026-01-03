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
    public final double PGAIN = 0.04;



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
    public RobotHardware(LinearOpMode opMode, boolean auto) {
        opMode_ = opMode;
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

        lFDriveSpeed = Math.max(lFDriveSpeed, -1);
        lBDriveSpeed = Math.max(lBDriveSpeed, -1);
        rFDriveSpeed = Math.max(rFDriveSpeed, -1);
        rBDriveSpeed = Math.max(rBDriveSpeed, -1);
        calculateDrive();
    }

    public void addAlignPower(double leftFront, double leftBack, double rightBack, double rightFront){
        lFAlignSpeed = leftFront;
        lBAlignSpeed = leftBack;
        rFAlignSpeed = rightFront;
        rBAlignSpeed = rightBack;

        lFAlignSpeed = Math.max(lFAlignSpeed, -1);
        lBAlignSpeed = Math.max(lBAlignSpeed, -1);
        rFAlignSpeed = Math.max(rFAlignSpeed, -1);
        rBAlignSpeed = Math.max(rBAlignSpeed, -1);

        calculateDrive();
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
                } else{
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


    public void alignFree(double llx) {
        //Use the Limelight llx to fine tune without stoping the driver
        if (llx != -999){
            turnToFree(robotHeadingAngles() - llx, .6);
        }
    }

    public void align(double llx){
        //Use the Limelight llx to fine tune
        if(llx != -999) {
            turnTo(robotHeadingAngles() - llx, .6);
        }
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
        double heading = robotHeadingAngles();

        mult = Math.abs(Math.abs(degrees) - Math.abs(heading)) * PGAIN;
        while(!( degrees - TURN_THRESH < heading && heading < degrees + TURN_THRESH) && opMode_.opModeIsActive()) {
            heading = robotHeadingAngles();
            if (degrees > 0) {
                oppDeg = degrees - 180;
                if (oppDeg < heading && heading < degrees) {
                    addAlignPower(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1), Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                } else  if(degrees < heading && heading < oppDeg){
                    addAlignPower(Math.min(speed * mult, 1), Math.min(speed * mult, 1), Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                } else {
                    addAlignPower(0,0,0,0);
                }
            } else {
                oppDeg = degrees + 180;
                if (degrees < heading && heading < oppDeg) {
                    addAlignPower(Math.min(speed * mult, 1), Math.min(speed * mult, 1), Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                } else if (oppDeg < heading && heading < degrees) {
                    addAlignPower(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1), Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                } else {
                    addAlignPower(0,0,0,0);
                }
            }
        }
        addAlignPower(0,0,0,0);
    }
}
