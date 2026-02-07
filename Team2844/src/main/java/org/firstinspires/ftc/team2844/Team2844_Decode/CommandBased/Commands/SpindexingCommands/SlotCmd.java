package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

import java.util.concurrent.TimeUnit;

public class SlotCmd extends CommandBase {

    SpindexerSubsystem spindexerSubsystem;
    KickSubsystem kickSubsystem;
    ElapsedTime timer;
    int slot;
    int desiredSlot;

    public SlotCmd(SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem, int desiredSlot){
        this.spindexerSubsystem = spindexerSubsystem;
        this.kickSubsystem = kickSubsystem;
        addRequirements(spindexerSubsystem, kickSubsystem);

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.desiredSlot = desiredSlot;
    }

    @Override
    public void initialize() {
        slot = spindexerSubsystem.getSlot();
        timer.reset();
        if(slot - desiredSlot < 0){
            kickSubsystem.runKickerSpin();
            kickSubsystem.runSFeedBackward();
        } else {
            kickSubsystem.runKickerSpinBackwards();
            kickSubsystem.runSFeedForward();
        }

        spindexerSubsystem.runToSlot(desiredSlot);
    }

    @Override
    public boolean isFinished() {
        double waitTime = (Math.abs(slot-desiredSlot))*300 + 750;
        if(waitTime < timer.startTime()) {
            return true;
        } else {
            return false;
        }
    }


    @Override
    public void end(boolean interrupted) {
        kickSubsystem.stopKickerSpin();
        kickSubsystem.stopSFeed();
    }
}
