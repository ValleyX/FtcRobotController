package org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardware {
    LinearOpMode opMode_;
    private final ElapsedTime runtime = new ElapsedTime();

    //motors and servos
    public Servo classSer = null;
    public DcMotorEx yawMotor;

    //sensor
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


}
