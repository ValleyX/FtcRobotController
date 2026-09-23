package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.ActivateIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands.TransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;

public class FullTransferAct implements Action {
    ShooterFeedSubsystem shooterFeedSubsystem;
    IntakeSubsystem intakeSubsystem;
    KickSubsystem kickSubsystem;
    public FullTransferAct(ShooterFeedSubsystem shooterFeedSubsystem, IntakeSubsystem intakeSubsystem, KickSubsystem kickSubsystem){
        this.shooterFeedSubsystem = shooterFeedSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.kickSubsystem = kickSubsystem;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        new ParallelAction(
                new CommandAction(new TransferCmd(shooterFeedSubsystem)),
                new CommandAction(new ActivateIntakeCmd(intakeSubsystem)),
                new UptakeAutoAct(kickSubsystem, () ->shooterFeedSubsystem.topBroken())
        ).run(telemetryPacket);
        return false;
    }
}
