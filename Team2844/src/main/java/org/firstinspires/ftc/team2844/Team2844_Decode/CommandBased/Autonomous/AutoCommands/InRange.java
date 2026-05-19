package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.StopTransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.TransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.StopUptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.UptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;

public class InRange implements Action {
    ShooterFeedSubsystem shooterFeedSubsystem;
    KickSubsystem kickSubsystem;
    ShooterSubsystem shooterSubsystem;
    public InRange(ShooterFeedSubsystem shooterFeedSubsystem, KickSubsystem kickSubsystem, ShooterSubsystem shooterSubsystem){
        this.shooterFeedSubsystem = shooterFeedSubsystem;
        this.kickSubsystem = kickSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if(shooterSubsystem.inRange()) {
            new ParallelAction(
                    new CommandAction(new TransferCmd(shooterFeedSubsystem)),
                    new CommandAction(new UptakeCmd(kickSubsystem))
            );
        } else {
            new ParallelAction(
                    new CommandAction(new StopTransferCmd(shooterFeedSubsystem)),
                    new CommandAction(new StopUptakeCmd(kickSubsystem))
            );
        }
        return false;
    }
}
