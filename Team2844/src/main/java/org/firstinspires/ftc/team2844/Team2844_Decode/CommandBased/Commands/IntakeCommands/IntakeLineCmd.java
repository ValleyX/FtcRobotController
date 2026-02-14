package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
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

        addRequirements(intakeSubsystem, shooterFeedSubsystem, shooterFeedSubsystem);
    }

    @Override
    public void execute() {
        boolean ballInBeam = intakeSubsystem.ballInBeam();
        boolean topBroken = shooterFeedSubsystem.topBroken();
        if(!(ballInBeam && topBroken)){
            intakeSubsystem.activate(Constants.INTAKE_SPEED);
            if(!topBroken){
//                new ParallelCommandGroup(new UptakeCmd(kickSubsystem), new TransferCmd(shooterFeedSubsystem));
                kickSubsystem.rotateKickerDown();
                kickSubsystem.runKickerSpin();
                kickSubsystem.runSFeedForward();
                shooterFeedSubsystem.runTFeedForward();
            } else {
//                new ParallelCommandGroup(new StopUptakeCmd(kickSubsystem), new StopTransferCmd(shooterFeedSubsystem));
                intakeSubsystem.stop();
                kickSubsystem.rotateKickerUp();
                kickSubsystem.stopKickerSpin();
                kickSubsystem.stopSFeed();
                shooterFeedSubsystem.stopTFeed();
            }

        } else {
            //new ParallelCommandGroup(new StopUptakeCmd(kickSubsystem), new StopTransferCmd(shooterFeedSubsystem), new StopIntakeCmd(intakeSubsystem));
            intakeSubsystem.stop();
            kickSubsystem.rotateKickerUp();
            kickSubsystem.stopKickerSpin();
            kickSubsystem.stopSFeed();
            shooterFeedSubsystem.stopTFeed();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
