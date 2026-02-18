package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands;

import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

public class RunSmartSortShootAutoCmd extends CommandBase {
    SmartSortShootAutoCmd smartSortShootAutoCmd;
    SpindexerSubsystem spindexerSubsystem;
    KickSubsystem kickSubsystem;
    ShooterFeedSubsystem shooterFeedSubsystem;
    ShooterSubsystem shooterSubsystem;
    SensorSubsystem sensorSubsystem;
    AimSubsystem aimSubsystem;
    IntakeSubsystem intakeSubsystem;
    DriveSubsystem driveSubsystem;
    Vector2d vector;
    double heading;
    Telemetry telemetry;

    public RunSmartSortShootAutoCmd(ShooterSubsystem shooterSubsystem, ShooterFeedSubsystem shooterFeedSubsystem, SensorSubsystem sensorSubsystem, AimSubsystem aimSubsystem, SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem, Vector2d vector, double heading, Telemetry telemetry){
        this.spindexerSubsystem = spindexerSubsystem;
        this.kickSubsystem = kickSubsystem;
        this.shooterFeedSubsystem = shooterFeedSubsystem;
        this.shooterSubsystem =  shooterSubsystem;
        this.sensorSubsystem =  sensorSubsystem;
        this.aimSubsystem    =  aimSubsystem;
        this.intakeSubsystem =  intakeSubsystem;
        this.driveSubsystem  =  driveSubsystem;
        this.vector          =  vector;
        this.heading         =  heading;
        this.telemetry = telemetry;
    }

    @Override
    public void initialize() {
        smartSortShootAutoCmd = new SmartSortShootAutoCmd( shooterSubsystem, shooterFeedSubsystem, sensorSubsystem, aimSubsystem, spindexerSubsystem, kickSubsystem, intakeSubsystem, driveSubsystem, vector, heading, telemetry);
    }

    @Override
    public void execute() {
        smartSortShootAutoCmd.schedule();
    }

    @Override
    public boolean isFinished() {
        return smartSortShootAutoCmd.isFinished();
    }
}
