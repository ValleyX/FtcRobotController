package org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardware {
    LinearOpMode opMode_;
    private final ElapsedTime runtime = new ElapsedTime();

    //motors and servos
    public Servo classSer = null;

    //sensor
    public NormalizedColorSensor colorSensor;
    public NormalizedRGBA colors;
    public float[] hsvValues;
    double gain = 2.5;

    public RobotHardware(LinearOpMode opMode) {
        opMode_ = opMode;
        //servos
        classSer = opMode_.hardwareMap.get(Servo.class, "classSer");
        classSer.setPosition(0.0);

        //motor hardwaremaps and stuffs

        //sensor hardware maps and stuffs
        colorSensor = opMode_.hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        colorSensor.setGain((float)gain);


    }

    /**
     * Get Colors
     * it gets the colors as a double array
     * @return the double array of rgb vaules (red, green, blue)
     */
    public double[] getColors(){
        double[] colorReturn = {colors.red, colors.green, colors.blue};
        return colorReturn;
    }

    public void setGain(double newGain){
        gain = newGain;
    }

    public double getGain(){
        return gain;
    }
}
