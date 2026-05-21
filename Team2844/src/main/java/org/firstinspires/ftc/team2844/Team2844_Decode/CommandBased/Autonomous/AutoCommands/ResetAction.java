package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.StopIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.StopTransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.StopUptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;


public class ResetAction implements Action {
    ShooterFeedSubsystem shooterFeedSubsystem;
    KickSubsystem kickSubsystem;
    IntakeSubsystem intakeSubsystem;
    public ResetAction(ShooterFeedSubsystem shooterFeedSubsystem, KickSubsystem kickSubsystem, IntakeSubsystem intakeSubsystem){
        this.shooterFeedSubsystem = shooterFeedSubsystem;
        this.kickSubsystem = kickSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        new ParallelAction(
                new CommandAction(new StopIntakeCmd(intakeSubsystem)),
                new CommandAction(new StopTransferCmd(shooterFeedSubsystem)),
                new CommandAction(new StopUptakeCmd(kickSubsystem))
        ).run(telemetryPacket);
        return false;
    }
}
