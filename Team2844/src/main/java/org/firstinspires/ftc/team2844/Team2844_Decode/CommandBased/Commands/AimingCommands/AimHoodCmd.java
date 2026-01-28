package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

public class AimHoodCmd extends CommandBase {
    AimSubsystem aimSubsystem;

    public AimHoodCmd(AimSubsystem aimSubsystem){
        this.aimSubsystem = aimSubsystem;
        addRequirements(aimSubsystem);
    }

    @Override
    public void execute(){
        aimSubsystem.aimHood(0.0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
