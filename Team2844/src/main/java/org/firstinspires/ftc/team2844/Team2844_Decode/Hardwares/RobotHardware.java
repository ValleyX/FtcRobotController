package org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardware {
    LinearOpMode opMode_;
    private final ElapsedTime runtime = new ElapsedTime();

    //motors and servos
    public Servo classSer = null;

    //sensor
    public NormalizedColorSensor colorSensor = null;
    double gain = 2.0;

    public RobotHardware(LinearOpMode opMode) {
        opMode_ = opMode;
        //servos
        classSer = opMode_.hardwareMap.get(Servo.class, "classSer");

        //motor hardwaremaps and stuffs

        //sensor hardware maps and stuffs
        colorSensor = opMode_.hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        colorSensor.setGain((float)gain);


    }

    public double[] getColors(){
       // return colorSensor.getNormalizedColors().to
    }

    public void setGain(double newGain){
        gain = newGain;
    }

    public double getGain(){
        return gain;
    }
}
