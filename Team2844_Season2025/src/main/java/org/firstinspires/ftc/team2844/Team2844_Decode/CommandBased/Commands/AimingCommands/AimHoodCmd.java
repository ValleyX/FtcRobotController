package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

public class AimHoodCmd extends CommandBase {
    AimSubsystem aimSubsystem;
    DriveSubsystem driveSubsystem;
    SensorSubsystem sensorSubsystem;

    public AimHoodCmd(AimSubsystem aimSubsystem, SensorSubsystem sensorSubsystem, DriveSubsystem driveSubsystem){
        this.aimSubsystem = aimSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.sensorSubsystem = sensorSubsystem;
        addRequirements(aimSubsystem);
    }

    @Override
    public void execute(){
        aimSubsystem.aimHood(driveSubsystem.hoodLinReg(sensorSubsystem.getPipeline()));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
