package org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.Limelight3A;
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

    //motors and servos
    public Servo classSer = null;
    public DcMotorEx yawMotor;

    //drive motors
    public DcMotor rightBackDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftFrontDrive;
    //sensor and IMU
    public IMU imu;
    public NormalizedColorSensor colorSensor;
    public NormalizedRGBA colors;
    double gain = 2.5;

    public Limelight3A limelight;


    public RobotHardware(LinearOpMode opMode) {
        opMode_ = opMode;
        //servos
        classSer = opMode_.hardwareMap.get(Servo.class, "classSer");
        classSer.setPosition(0.5);

        //motor hardwaremaps and stuffs
        yawMotor = opMode_.hardwareMap.get(DcMotorEx.class, "yawMotor");

        //drive motor hardwarmaps
        rightFrontDrive = opMode_.hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = opMode_.hardwareMap.get(DcMotor.class, "rightBack");
        leftFrontDrive = opMode_.hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = opMode_.hardwareMap.get(DcMotor.class, "leftBack");

        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        //sensor hardware maps and stuffs
        colorSensor = opMode_.hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colors = colorSensor.getNormalizedColors();
        colorSensor.setGain((float)gain);

        //limelight stuffs and hardware map
        limelight = opMode_.hardwareMap.get(Limelight3A.class, "limelight");

    }

    /**
     * Get Colors
     * it gets the colors as a double array
     * @return the double array of rgb vaules (red, green, blue)
     */
    public float[] getColors(){
        colors = colorSensor.getNormalizedColors();
        float[] colorReturn = {colors.red, colors.green, colors.blue};
        return colorReturn;
    }

    public void setGain(double newGain){
        gain = newGain;
    }

    public double getGain(){
        return gain;
    }

    public void moveServo(double pos){
        classSer.setPosition(pos);
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

}
