package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.FallingEdgeTrigger;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

import java.util.function.DoubleSupplier;

public class UptakeLoop extends RunCommand {

    ShooterSubsystem shooterSubsystem;
    ShooterFeedSubsystem shooterFeedSubsystem;
    SpindexerSubsystem spindexerSubsystem;
    IntakeSubsystem intakeSubsystem;
    KickSubsystem kickSubsystem;
    DoubleSupplier velocity;
    Telemetry telemetry;


    public UptakeLoop(ShooterSubsystem shooterSubsystem, ShooterFeedSubsystem shooterFeedSubsystem,
                      SpindexerSubsystem spindexerSubsystem, IntakeSubsystem intakeSubsystem,
                      KickSubsystem kickSubsystem, DoubleSupplier velocity, Telemetry telemetry) {
        super(()->{},
                shooterFeedSubsystem,
                spindexerSubsystem,
                intakeSubsystem,
                kickSubsystem);


        this.shooterSubsystem = shooterSubsystem;
        this.kickSubsystem = kickSubsystem;
        this.spindexerSubsystem = spindexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterFeedSubsystem = shooterFeedSubsystem;
        this.velocity = velocity;
        this.telemetry = telemetry;
    }

    @Override
    public void execute() {

    }
}
