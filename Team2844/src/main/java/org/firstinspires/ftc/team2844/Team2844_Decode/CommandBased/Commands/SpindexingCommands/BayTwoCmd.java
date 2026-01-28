package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

public class BayTwoCmd extends CommandBase {

    SpindexerSubsystem spindexerSubsystem;

    public BayTwoCmd(SpindexerSubsystem spindexerSubsystem){
        this.spindexerSubsystem = spindexerSubsystem;
        addRequirements(spindexerSubsystem);
    }

    @Override
    public void initialize() {
        spindexerSubsystem.runToBayTwo();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}