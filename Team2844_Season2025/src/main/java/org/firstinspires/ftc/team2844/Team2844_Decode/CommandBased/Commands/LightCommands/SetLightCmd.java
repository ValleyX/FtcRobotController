package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.LightCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ExtraSubsystems.LightSubsystem;

public class SetLightCmd extends CommandBase {
    LightSubsystem lightSubsystem;
    int index;
    double color;
    public SetLightCmd(LightSubsystem lightSubsystem, int index, double color){
        this.lightSubsystem = lightSubsystem;
        this.index = index;
        this.color = color;
    }

    @Override
    public void initialize() {
        lightSubsystem.setLight(index, color);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
