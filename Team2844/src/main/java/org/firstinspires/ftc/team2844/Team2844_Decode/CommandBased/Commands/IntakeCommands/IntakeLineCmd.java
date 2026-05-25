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

    boolean ballInBeam;
    boolean topBroken;
    boolean bayOne;
    boolean hasBeenBayOne;
    ElapsedTime timer;

    boolean startedFull;

    public IntakeLineCmd(ShooterFeedSubsystem shooterFeedSubsystem, IntakeSubsystem intakeSubsystem, SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.shooterFeedSubsystem = shooterFeedSubsystem;
        this.kickSubsystem = kickSubsystem;
        this.spindexerSubsystem = spindexerSubsystem;
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        startedFull = shooterFeedSubsystem.topBroken();
        addRequirements(intakeSubsystem, shooterFeedSubsystem, shooterFeedSubsystem);
    }

    @Override
    public void initialize() {
        hasBeenBayOne = false;
    }

    @Override
    public void execute() {
        boolean ballInBeam = intakeSubsystem.ballInBeam();
        boolean topBroken = shooterFeedSubsystem.topBroken();

        if (ballInBeam) {
            hasBeenBayOne = true;
        }


        if (!topBroken) {
            intakeSubsystem.activate(Constants.INTAKE_SPEED);
            kickSubsystem.rotateKickerDown();
            kickSubsystem.runKickerSpin();
            kickSubsystem.runSFeedForward();
            timer.reset();
        } else {
            if(timer.time() < 450 && !startedFull){
                kickSubsystem.rotateKickerDownIntake();
                kickSubsystem.runKickerSpin();
                kickSubsystem.runSFeedForward();
            } else {
                kickSubsystem.rotateKickerUp();
                kickSubsystem.stopKickerSpin();
                kickSubsystem.runSFeedBackward();
            }

            intakeSubsystem.activate(Constants.INTAKE_SPEED);
        }

    }



    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
        kickSubsystem.rotateKickerUp();
        kickSubsystem.stopKickerSpin();
        kickSubsystem.stopSFeed();
        shooterFeedSubsystem.stopTFeed();
    }
}
