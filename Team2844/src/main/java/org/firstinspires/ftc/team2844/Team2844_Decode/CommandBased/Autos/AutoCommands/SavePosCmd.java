package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.SavedVars;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;

public class SavePosCmd extends CommandBase {
    DriveSubsystem driveSubsystem;
    SensorSubsystem sensorSubsystem;

    public SavePosCmd(DriveSubsystem driveSubsystem, SensorSubsystem sensorSubsystem){
        this.driveSubsystem = driveSubsystem;
        this.sensorSubsystem = sensorSubsystem;
    }

    @Override
    public void execute(){
        SavedVars.startingX = driveSubsystem.getBotX();
        SavedVars.startingY = driveSubsystem.getBotY();
        SavedVars.startingHeading = driveSubsystem.getRobotHeading();
        SavedVars.pattern = sensorSubsystem.getPattern();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
