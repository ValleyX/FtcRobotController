package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ExtraSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;

import java.util.concurrent.TimeUnit;

public class LightSubsystem extends SubsystemBase {
    /**The array of all the lights, it goes in this order: 0: top, 1: mid, 2: bot, 3: left, 4: right*/
    CRServo[] lightArray;

    ElapsedTime timer;

    double[] colors = {0.0, 0.0, 0.0, 0.0, 0.0};

    double time;

    public LightSubsystem(CRServo[] lightArray){
        this.lightArray = lightArray;
        timer = new ElapsedTime();
        timer.reset();
    }

    public void setLight(int index, double color){
        colors[index] = color;
        lightArray[index].setPower(color);
    }

    public double getColor(int index){
        return colors[index];
    }
    @Override
    public void periodic() {
        //TOP
        lightArray[Constants.TOPL_INDEX].setPower(colors[Constants.TOPL_INDEX]);

        //MID
        lightArray[Constants.MIDL_INDEX].setPower(colors[Constants.MIDL_INDEX]);

        //BOT
        lightArray[Constants.BOTL_INDEX].setPower(colors[Constants.BOTL_INDEX]);

        //LEFT
        lightArray[Constants.LEFTL_INDEX].setPower(colors[Constants.LEFTL_INDEX]);

        //RIGHT
        lightArray[Constants.RIGHTL_INDEX].setPower(colors[Constants.RIGHTL_INDEX]);

    }
}
