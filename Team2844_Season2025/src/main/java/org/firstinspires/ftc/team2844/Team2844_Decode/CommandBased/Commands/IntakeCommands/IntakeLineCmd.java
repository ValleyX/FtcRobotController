package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    ElapsedTime timer;

    boolean startedFull;

    public IntakeLineCmd(ShooterFeedSubsystem shooterFeedSubsystem, IntakeSubsystem intakeSubsystem, SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.shooterFeedSubsystem = shooterFeedSubsystem;
        this.kickSubsystem = kickSubsystem;
        this.spindexerSubsystem = spindexerSubsystem;
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        addRequirements(intakeSubsystem, shooterFeedSubsystem, shooterFeedSubsystem);
    }

    @Override
    public void initialize() {
        startedFull = shooterFeedSubsystem.topBroken();
        intakeSubsystem.activate(Constants.INTAKE_SPEED);
        kickSubsystem.runKickerSpin();
        kickSubsystem.runSFeedForward();
    }

    @Override
    public void execute() {
        boolean topBroken = shooterFeedSubsystem.topBroken();
        if (!topBroken) {
            kickSubsystem.rotateKickerDown();
            timer.reset();
        } else {
            if(timer.time() < 450 && !startedFull){
                kickSubsystem.rotateKickerDownIntake();
            } else {
                kickSubsystem.stopKickerSpin();
                kickSubsystem.runSFeedBackward();
            }
        }

    }



    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
        //kickSubsystem.rotateKickerUp();
        kickSubsystem.stopKickerSpin();
        kickSubsystem.stopSFeed();
        shooterFeedSubsystem.stopTFeed();
    }
}
