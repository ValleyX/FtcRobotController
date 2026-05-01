package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

public class PreviousSlotCmd extends CommandBase {

    SpindexerSubsystem spindexerSubsystem;
    KickSubsystem kickSubsystem;

    public PreviousSlotCmd(SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem){
        this.spindexerSubsystem = spindexerSubsystem;
        this.kickSubsystem = kickSubsystem;
    }

    @Override
    public void initialize() {
        new SlotCmd(spindexerSubsystem, kickSubsystem, spindexerSubsystem.getSlot()-1).schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
