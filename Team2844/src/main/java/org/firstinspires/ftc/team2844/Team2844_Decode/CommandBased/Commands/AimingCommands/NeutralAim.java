package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

public class NeutralAim extends CommandBase {

    AimSubsystem aimSubsystem;

    public NeutralAim (AimSubsystem aimSubsystem){
        this.aimSubsystem = aimSubsystem;
    }

    @Override
    public void initialize() {
        aimSubsystem.aimTurret(90.0);
        aimSubsystem.aimHood(0.0);
    }
}
