package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

import java.util.concurrent.TimeUnit;

public class SlotWaitCmd extends CommandBase {

    SpindexerSubsystem spindexerSubsystem;
    ElapsedTime timer;
    int slot;
    int desiredSlot;
    boolean finished = false;

    public SlotWaitCmd(SpindexerSubsystem spindexerSubsystem, int desiredSlot){
        this.spindexerSubsystem = spindexerSubsystem;
        addRequirements(spindexerSubsystem);

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        if(0 <= desiredSlot && desiredSlot < Constants.SLOT_ARRAY.length) {
            this.desiredSlot = desiredSlot;
        } else if (0 > desiredSlot) {
            this.desiredSlot = 0;
        } else if(desiredSlot >= Constants.SLOT_ARRAY.length){
            this.desiredSlot = Constants.SLOT_ARRAY.length-1;
        }
    }

    @Override
    public void initialize() {
        slot = spindexerSubsystem.getSlot();
        timer.reset();
        spindexerSubsystem.runToSlot(desiredSlot);
    }


    public void execute(){
        double waitTime = (Math.abs(slot-desiredSlot))*300 + 250;
        if(timer.time(TimeUnit.MILLISECONDS) < waitTime) {
            finished = false;
        } else {
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
