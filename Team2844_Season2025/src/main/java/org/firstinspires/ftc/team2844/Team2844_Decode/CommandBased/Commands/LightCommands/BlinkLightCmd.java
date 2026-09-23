package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.LightCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ExtraSubsystems.LightSubsystem;

import java.util.concurrent.TimeUnit;

public class BlinkLightCmd extends CommandBase {
    ElapsedTime totalTimer;
    ElapsedTime blinkTimer;
    LightSubsystem lightSubsystem;
    int index;
    double color1, color2;
    long blinkTime;
    long totalTime;
    double prevColor;
    /**
     * BlinkLightCmd
     * @param lightSubsystem
     * @param index
     * @param color1 The first color to blink
     * @param color2 The second color to blink
     * @param blinkTime The length of each blink
     * @param totalTime The total length of blinking
     * */
    public BlinkLightCmd(LightSubsystem lightSubsystem, int index, double color1, double color2, long blinkTime, long totalTime){
        totalTimer = new ElapsedTime();
        totalTimer.reset();
        blinkTimer = new ElapsedTime();
        blinkTimer.reset();
        this.lightSubsystem = lightSubsystem;
        this.index = index;
        this.color1 = color1;
        this.color2 = color2;
        this.blinkTime = blinkTime;
        this.totalTime = totalTime;
        prevColor = lightSubsystem.getColor(index);
    }

    @Override
    public void initialize() {
        totalTimer.reset();
        blinkTimer.reset();
        lightSubsystem.setLight(index, color1);
    }

    @Override
    public void execute() {
        if(blinkTimer.time(TimeUnit.MILLISECONDS) < blinkTime){
            lightSubsystem.setLight(index, color1);
        } else if(blinkTimer.time(TimeUnit.MILLISECONDS) < blinkTime*2){
            lightSubsystem.setLight(index, color2);
        } else {
            blinkTimer.reset();
        }
    }

    @Override
    public boolean isFinished(){
        return totalTimer.time(TimeUnit.MILLISECONDS) > totalTime;
    }

    @Override
    public void end(boolean interrupted) {
        lightSubsystem.setLight(index, prevColor);
    }
}
