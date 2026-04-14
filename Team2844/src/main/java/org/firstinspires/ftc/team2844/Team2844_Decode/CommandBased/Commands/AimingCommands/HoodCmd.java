package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

public class HoodCmd extends CommandBase {
    AimSubsystem aimSubsystem;
    double pos;
    public HoodCmd(AimSubsystem aimSubsystem, double pos){
        this.aimSubsystem = aimSubsystem;
        this.pos = pos;
    }

    @Override
    public void initialize() {
        aimSubsystem.aimHood(pos);
    }
}
