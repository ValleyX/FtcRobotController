package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

import java.util.function.DoubleSupplier;

public class HoodCmd extends CommandBase {
    AimSubsystem aimSubsystem;
    DoubleSupplier pos;
    public HoodCmd(AimSubsystem aimSubsystem, DoubleSupplier pos){
        this.aimSubsystem = aimSubsystem;
        this.pos = pos;

        addRequirements(aimSubsystem);
    }

    @Override
    public void initialize() {
        aimSubsystem.aimHood(pos.getAsDouble());
    }
}
