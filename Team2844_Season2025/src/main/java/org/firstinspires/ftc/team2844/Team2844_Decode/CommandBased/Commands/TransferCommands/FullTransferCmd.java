package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands.CommandAction;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands.UptakeAutoAct;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.ActivateIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;

public class FullTransferCmd extends ParallelCommandGroup {
    public FullTransferCmd(ShooterFeedSubsystem shooterFeedSubsystem, IntakeSubsystem intakeSubsystem, KickSubsystem kickSubsystem){
        addCommands(
                new TransferCmd(shooterFeedSubsystem),
                new ActivateIntakeCmd(intakeSubsystem),
                new FullUptakeCmd(kickSubsystem, shooterFeedSubsystem)
        );
    }
}
