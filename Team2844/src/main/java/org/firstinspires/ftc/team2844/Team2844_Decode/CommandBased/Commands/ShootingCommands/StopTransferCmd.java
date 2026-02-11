package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;

public class StopTransferCmd extends CommandBase {
    ShooterFeedSubsystem shooterFeedSubsystem;

    public StopTransferCmd(ShooterFeedSubsystem shooterFeedSubsystem){
        this.shooterFeedSubsystem = shooterFeedSubsystem;
    }

    @Override
    public void execute() {
        shooterFeedSubsystem.stopTFeed();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
