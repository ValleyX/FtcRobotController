package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;

public class FullExtakeCmd extends CommandBase {

    IntakeSubsystem intakeSubsystem;
    ShooterFeedSubsystem shooterFeedSubsystem;
    KickSubsystem kickSubsystem;

    public FullExtakeCmd(IntakeSubsystem intakeSubsystem, ShooterFeedSubsystem shooterFeedSubsystem, KickSubsystem kickSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        this.shooterFeedSubsystem = shooterFeedSubsystem;
        this.kickSubsystem = kickSubsystem;
        addRequirements(intakeSubsystem, shooterFeedSubsystem, kickSubsystem);
    }

    @Override
    public void initialize() {
        shooterFeedSubsystem.runTFeedBackward();
        intakeSubsystem.activate(-1.0);
        kickSubsystem.rotateKickerUp();
        kickSubsystem.runSFeedBackward();
    }

    @Override
    public void end(boolean interrupted) {
        shooterFeedSubsystem.stopTFeed();
        intakeSubsystem.stop();
        kickSubsystem.rotateKickerUp();
        kickSubsystem.stopSFeed();
    }
}
