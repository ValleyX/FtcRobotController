package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

public class AimHoodCmd extends CommandBase {
    AimSubsystem aimSubsystem;
    SensorSubsystem sensorSubsystem;

    public AimHoodCmd(AimSubsystem aimSubsystem, SensorSubsystem sensorSubsystem){
        this.aimSubsystem = aimSubsystem;
        this.sensorSubsystem = sensorSubsystem;
        addRequirements(aimSubsystem, sensorSubsystem);
    }

    @Override
    public void execute(){
        double tx = sensorSubsystem.getTx();
        if(tx != Constants.NO_LL){
            aimSubsystem.aimHood(sensorSubsystem.hoodLinReg());
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
