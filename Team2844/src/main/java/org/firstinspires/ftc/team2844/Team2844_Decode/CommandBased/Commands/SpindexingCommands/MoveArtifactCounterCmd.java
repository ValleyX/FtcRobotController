package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

public class MoveArtifactCounterCmd extends SlotCmd{
    boolean bay1;
    boolean bay2;
    boolean bay3;


    /**MUST HAVE A TIMEOUT TO PROTECT FROM STALLING BECAUSE OF INVALID SLOTS*/
    public MoveArtifactCounterCmd(SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem){
        super(spindexerSubsystem, kickSubsystem, spindexerSubsystem.getSlot()-1);
        bay1 = spindexerSubsystem.ballInBayOne();
        bay2 = spindexerSubsystem.ballInBayTwo();
        bay3 = spindexerSubsystem.ballInBayThree();
    }

    @Override
    public void initialize() {
        desiredSlot = spindexerSubsystem.getSlot()-1;
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        if(bay1 && bay2 && !bay3){
            return spindexerSubsystem.ballInBayThree() && !spindexerSubsystem.ballInBayTwo();
        } else if(bay1 && !bay2 && !bay3){
            return spindexerSubsystem.ballInBayThree() && !spindexerSubsystem.ballInBayOne();
        } else if(!bay1 && bay2 && !bay3){
            return spindexerSubsystem.ballInBayOne() && !spindexerSubsystem.ballInBayTwo();
        } else if(!bay1 && bay2 && bay3){
            return spindexerSubsystem.ballInBayOne() && !spindexerSubsystem.ballInBayThree();
        } else if(bay1 && !bay2 && bay3) {
            return spindexerSubsystem.ballInBayTwo() && !spindexerSubsystem.ballInBayOne();
        } else
            return (bay1 && bay2 && bay3) || (!bay1 && !bay2 && !bay3);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
