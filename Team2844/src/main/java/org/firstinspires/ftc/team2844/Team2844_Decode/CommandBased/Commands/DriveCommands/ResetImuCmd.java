package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;

public class ResetImuCmd extends CommandBase {
    SensorSubsystem sensorSubsystem;
    public ResetImuCmd(SensorSubsystem sensorSubsystem){
        this.sensorSubsystem = sensorSubsystem;
    }

    @Override
    public void initialize() {
        sensorSubsystem.resetIMU();
    }
}
