package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

public class NextSlotCmd extends CommandBase {

    SpindexerSubsystem spindexerSubsystem;
    KickSubsystem kickSubsystem;

    public NextSlotCmd(SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem){
        this.spindexerSubsystem = spindexerSubsystem;
        this.kickSubsystem = kickSubsystem;
    }

    @Override
    public void initialize() {
        new SlotCmd(spindexerSubsystem, kickSubsystem, spindexerSubsystem.getSlot()+1);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
