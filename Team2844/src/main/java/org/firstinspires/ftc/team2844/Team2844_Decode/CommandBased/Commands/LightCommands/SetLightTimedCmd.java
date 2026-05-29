package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.LightCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ExtraSubsystems.LightSubsystem;

import java.util.concurrent.TimeUnit;

public class SetLightTimedCmd extends CommandBase {
    ElapsedTime timer;
    LightSubsystem lightSubsystem;
    int index;
    double color;
    long milliseconds;
    double prevColor;
    public SetLightTimedCmd(LightSubsystem lightSubsystem, int index, double color, long milliseconds){
        timer = new ElapsedTime();
        timer.reset();
        this.lightSubsystem = lightSubsystem;
        this.index = index;
        this.color = color;
        this.milliseconds = milliseconds;
        prevColor = lightSubsystem.getColor(index);
    }

    @Override
    public void initialize() {
        timer.reset();
        lightSubsystem.setLight(index, color);
    }

    @Override
    public boolean isFinished(){
        return timer.time(TimeUnit.MILLISECONDS) > milliseconds;
    }

    @Override
    public void end(boolean interrupted) {
        lightSubsystem.setLight(index, prevColor);
    }
}
