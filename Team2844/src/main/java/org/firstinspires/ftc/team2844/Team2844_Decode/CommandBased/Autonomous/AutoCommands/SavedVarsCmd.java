package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.SavedVars;

import java.util.function.Supplier;

public class SavedVarsCmd extends CommandBase {
    double x;
    double y;
    double heading;
    public SavedVarsCmd(Supplier<Pose2d> pose){
        this.x = pose.get().position.x;
        this.y = pose.get().position.y;
        this.heading = pose.get().heading.toDouble();
    }

    public void initialize(){
        SavedVars.startingX = x;
        SavedVars.startingY = y;
        SavedVars.startingHeading = heading;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
