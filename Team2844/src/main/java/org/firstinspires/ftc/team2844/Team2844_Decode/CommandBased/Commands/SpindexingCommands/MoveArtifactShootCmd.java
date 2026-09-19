package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

public class MoveArtifactShootCmd extends CommandBase {
    SpindexerSubsystem spindexerSubsystem;
    boolean bay1;
    boolean bay2;
    boolean bay3;
    int desiredSlot;

    public MoveArtifactShootCmd(SpindexerSubsystem spindexerSubsystem){
        this.spindexerSubsystem = spindexerSubsystem;
        bay1 = spindexerSubsystem.ballInBayOne();
        bay2 = spindexerSubsystem.ballInBayTwo();
        bay3 = spindexerSubsystem.ballInBayThree();
    }

    @Override
    public void initialize() {
        bay1 = spindexerSubsystem.ballInBayOne();
        bay2 = spindexerSubsystem.ballInBayTwo();
        bay3 = spindexerSubsystem.ballInBayThree();
        desiredSlot = spindexerSubsystem.getSlot() -1;
        if (desiredSlot < 0) {
            desiredSlot = 0;
        } else if(Constants.SLOT_ARRAY.length <= desiredSlot){
            desiredSlot = Constants.SLOT_ARRAY.length-1;
        }
    }

    @Override
    public void execute() {
        spindexerSubsystem.runToShootSlot(desiredSlot);
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
}
