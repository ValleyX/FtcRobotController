package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;

public class TransferCmd extends CommandBase {
    ShooterFeedSubsystem shooterFeedSubsystem;

    public TransferCmd(ShooterFeedSubsystem shooterFeedSubsystem){
        this.shooterFeedSubsystem = shooterFeedSubsystem;
        addRequirements(shooterFeedSubsystem);
    }

    @Override
    public void initialize() {
        shooterFeedSubsystem.runTFeedForward();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
