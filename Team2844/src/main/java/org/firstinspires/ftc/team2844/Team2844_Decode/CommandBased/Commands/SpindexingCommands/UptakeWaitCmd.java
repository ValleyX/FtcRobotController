package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

public class UptakeWaitCmd extends CommandBase {
    KickSubsystem kickSubsystem;
    SpindexerSubsystem spindexerSubsystem;

    public UptakeWaitCmd(KickSubsystem kickSubsystem, SpindexerSubsystem spindexerSubsystem){
        this.kickSubsystem = kickSubsystem;
        this.spindexerSubsystem = spindexerSubsystem;
        addRequirements(kickSubsystem);
    }

    @Override
    public void initialize() {
        kickSubsystem.rotateKickerDown(); //TANNER!!!!!! I have an UptakeExtraCmd, so if you want it to Uptake extra, use that
        kickSubsystem.runKickerSpin();
        kickSubsystem.runSFeedForward();
    }


    @Override
    public boolean isFinished() {
        return !spindexerSubsystem.ballInBayOne();
    }
}
