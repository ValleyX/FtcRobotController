package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

public class UptakeCmd extends CommandBase {
    KickSubsystem kickSubsystem;
    SpindexerSubsystem spindexerSubsystem;

    public UptakeCmd(KickSubsystem kickSubsystem, SpindexerSubsystem spindexerSubsystem){
        this.kickSubsystem = kickSubsystem;
        this.spindexerSubsystem = spindexerSubsystem;
        addRequirements(kickSubsystem);
    }

    @Override
    public void initialize() {
        kickSubsystem.rotateKickerDown();
        kickSubsystem.runKickerSpin();
        kickSubsystem.runSFeedForward();
        // Extra kick logic? Yea that would probably go in here. or maybe in the shoot logic itself
        spindexerSubsystem.runToShootSlot(spindexerSubsystem.getSlot());
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}
