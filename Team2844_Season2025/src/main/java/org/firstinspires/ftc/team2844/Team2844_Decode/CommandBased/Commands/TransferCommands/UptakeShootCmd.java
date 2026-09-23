package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

public class UptakeShootCmd extends SequentialCommandGroup {
    public UptakeShootCmd(KickSubsystem kickSubsystem, SpindexerSubsystem spindexerSubsystem, ShooterFeedSubsystem shooterFeedSubsystem){
        addCommands(
                new UptakeWaitCmd(kickSubsystem, spindexerSubsystem),
                new UptakeExtraWaitCmd(kickSubsystem, shooterFeedSubsystem)
        );
    }
}
