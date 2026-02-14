package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

import java.util.concurrent.TimeUnit;

public class SlotCmd extends CommandBase {

    SpindexerSubsystem spindexerSubsystem;
    KickSubsystem kickSubsystem;
    ElapsedTime timer;
    int slot;
    int desiredSlot;
    boolean finished = false;

    public SlotCmd(SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem, int desiredSlot){
        this.spindexerSubsystem = spindexerSubsystem;
        this.kickSubsystem = kickSubsystem;
        addRequirements(spindexerSubsystem, kickSubsystem);

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        if(0 <= desiredSlot && desiredSlot < Constants.SLOT_ARRAY.length) {
            this.desiredSlot = desiredSlot;
        } else if (0 > desiredSlot) {
            this.desiredSlot = 0;
        } else if(desiredSlot > Constants.SLOT_ARRAY.length){
            this.desiredSlot = Constants.SLOT_ARRAY.length-1;
        }
    }

    @Override
    public void initialize() {
        slot = spindexerSubsystem.getSlot();
        timer.reset();
        if(slot - desiredSlot < 0){
            kickSubsystem.runKickerSpin();
            kickSubsystem.runSFeedBackward();
        } else if(slot - desiredSlot == 0){
            kickSubsystem.stopKickerSpin();
            kickSubsystem.stopSFeed();
        } else {
            kickSubsystem.runKickerSpinBackwards();
            kickSubsystem.runSFeedForward();
        }

        spindexerSubsystem.runToSlot(desiredSlot);
    }


    public void execute(){
        double waitTime = (Math.abs(slot-desiredSlot))*300 + 250;
        if(timer.time(TimeUnit.MILLISECONDS) < waitTime) {
            finished = false;
        } else {
            finished = true;
            kickSubsystem.stopSFeed();
            kickSubsystem.stopKickerSpin();
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }


    @Override
    public void end(boolean interrupted) {
        kickSubsystem.stopKickerSpin();
        kickSubsystem.stopSFeed();
    }
}
