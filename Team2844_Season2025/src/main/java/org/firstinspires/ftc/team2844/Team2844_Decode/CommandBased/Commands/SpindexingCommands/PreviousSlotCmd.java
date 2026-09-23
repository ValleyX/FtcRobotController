package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

public class PreviousSlotCmd extends SlotCmd {

    public PreviousSlotCmd(SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem){
        super(spindexerSubsystem, kickSubsystem, spindexerSubsystem.getSlot()-1);
    }

    @Override
    public void initialize() {
        if(spindexerSubsystem.getSlot()-1>=0){
            desiredSlot = spindexerSubsystem.getSlot()-1;
        }
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
