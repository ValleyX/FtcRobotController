package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

public class FirstFullSlotCmd extends SlotCmd {

    public FirstFullSlotCmd(SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem) {
        super(spindexerSubsystem, kickSubsystem, spindexerSubsystem.getFirstFull());
    }

    @Override
    public void initialize() {
        desiredSlot = spindexerSubsystem.getFirstFull();
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
