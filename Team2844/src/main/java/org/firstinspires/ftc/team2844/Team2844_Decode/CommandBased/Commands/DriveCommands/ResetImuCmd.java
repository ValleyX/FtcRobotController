package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;

public class ResetImuCmd extends CommandBase {
    DriveSubsystem driveSubsystem;
    double degrees;
    public ResetImuCmd(DriveSubsystem driveSubsystem, double degrees){
        this.driveSubsystem = driveSubsystem;
        this.degrees = degrees;
    }

    @Override
    public void initialize() {
        driveSubsystem.resetIMU(degrees);
    }
}
