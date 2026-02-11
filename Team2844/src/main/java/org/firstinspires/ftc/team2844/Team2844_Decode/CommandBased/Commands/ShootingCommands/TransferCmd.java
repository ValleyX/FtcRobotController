package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;

public class TransferCmd extends CommandBase {
    ShooterFeedSubsystem shooterFeedSubsystem;

    public TransferCmd(ShooterFeedSubsystem shooterFeedSubsystem){
        this.shooterFeedSubsystem = shooterFeedSubsystem;
    }

    @Override
    public void execute() {
        shooterFeedSubsystem.runTFeedForward();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
