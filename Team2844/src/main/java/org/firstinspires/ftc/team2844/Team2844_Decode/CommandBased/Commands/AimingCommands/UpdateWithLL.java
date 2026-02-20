/*package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;

public class UpdateWithLL extends CommandBase {

    DriveSubsystem driveSubsystem;
    SensorSubsystem sensorSubsystem;

    public UpdateWithLL(DriveSubsystem driveSubsystem, SensorSubsystem sensorSubsystem){
        this.driveSubsystem = driveSubsystem;
        this.sensorSubsystem = sensorSubsystem;
    }

    @Override
    public void execute() {
        double llX = sensorSubsystem.getBotXLL();
        double llY = sensorSubsystem.getBotYLL();

        double ppX = driveSubsystem.getBotX();
        double ppY = driveSubsystem.getBotY();

        if(llX != Constants.NO_LL && llY != Constants.NO_LL){
            if(){

            }
        }
    }
}*/
