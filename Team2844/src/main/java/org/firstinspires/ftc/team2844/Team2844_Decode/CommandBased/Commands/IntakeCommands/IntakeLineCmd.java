package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.StopUptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.UptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

public class IntakeLineCmd extends CommandBase {
    ShooterFeedSubsystem shooterFeedSubsystem;
    IntakeSubsystem intakeSubsystem;
    SpindexerSubsystem spindexerSubsystem;
    KickSubsystem kickSubsystem;

    public IntakeLineCmd(ShooterFeedSubsystem shooterFeedSubsystem, IntakeSubsystem intakeSubsystem, SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        this.shooterFeedSubsystem = shooterFeedSubsystem;
        this.kickSubsystem = kickSubsystem;
        this.spindexerSubsystem = spindexerSubsystem;

        addRequirements(intakeSubsystem, shooterFeedSubsystem);
    }

    @Override
    public void execute() {
        if(!(intakeSubsystem.ballInBeam() && spindexerSubsystem.ballInBayOne() && shooterFeedSubsystem.topBroken())){
            intakeSubsystem.activate(Constants.INTAKE_SPEED);
            if(!shooterFeedSubsystem.topBroken()){
                shooterFeedSubsystem.runTFeedForward();
                new UptakeCmd(kickSubsystem);
            } else {
                shooterFeedSubsystem.stopTFeed();
                new StopUptakeCmd(kickSubsystem);
            }

        } else {
            intakeSubsystem.stop();
            shooterFeedSubsystem.stopTFeed();
            new StopUptakeCmd(kickSubsystem);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
