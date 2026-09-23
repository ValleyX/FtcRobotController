package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.ActivateIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.StopIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands.FullTransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands.StopTransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands.TransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.StopUptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;

public class InRange implements Action {
    ShooterFeedSubsystem shooterFeedSubsystem;
    KickSubsystem kickSubsystem;
    ShooterSubsystem shooterSubsystem;
    IntakeSubsystem intakeSubsystem;
    public InRange(ShooterFeedSubsystem shooterFeedSubsystem, KickSubsystem kickSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem){
        this.shooterFeedSubsystem = shooterFeedSubsystem;
        this.kickSubsystem = kickSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if(shooterSubsystem.inRange()) {
            new FullTransferAct(shooterFeedSubsystem, intakeSubsystem, kickSubsystem).run(telemetryPacket);
        } else {
            new StopFullTransferAct(shooterFeedSubsystem, intakeSubsystem, kickSubsystem).run(telemetryPacket);
        }
        return true;
    }
}
