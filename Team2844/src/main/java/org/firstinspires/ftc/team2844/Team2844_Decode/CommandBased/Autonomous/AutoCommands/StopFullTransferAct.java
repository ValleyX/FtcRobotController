package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;

public class StopFullTransferAct implements Action {

    ShooterFeedSubsystem shooterFeedSubsystem;
    IntakeSubsystem intakeSubsystem;
    KickSubsystem kickSubsystem;

    public StopFullTransferAct(ShooterFeedSubsystem shooterFeedSubsystem, IntakeSubsystem intakeSubsystem, KickSubsystem kickSubsystem) {
        this.shooterFeedSubsystem = shooterFeedSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.kickSubsystem = kickSubsystem;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        shooterFeedSubsystem.stopTFeed();
        intakeSubsystem.stop();
        kickSubsystem.rotateKickerUp();
        kickSubsystem.stopKickerSpin();
        kickSubsystem.stopSFeed();

        return false;
    }
}
