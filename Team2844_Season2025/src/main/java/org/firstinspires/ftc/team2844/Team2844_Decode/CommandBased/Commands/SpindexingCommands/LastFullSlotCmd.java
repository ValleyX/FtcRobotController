package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

public class LastFullSlotCmd extends SlotCmd{
    public LastFullSlotCmd(SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem) {
        super(spindexerSubsystem, kickSubsystem, spindexerSubsystem.getLastFull());
    }

    @Override
    public void initialize() {
        desiredSlot = spindexerSubsystem.getLastFull();
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return spindexerSubsystem.ballInBayOne() && finished;
    }
}
