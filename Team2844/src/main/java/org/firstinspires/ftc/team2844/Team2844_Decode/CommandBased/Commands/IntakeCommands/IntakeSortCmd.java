package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.SlotCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

public class IntakeSortCmd extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private SpindexerSubsystem spindexerSubsystem;
    private KickSubsystem kickSubsystem;
    SlotCmd slotCmd;


    public IntakeSortCmd(IntakeSubsystem intakeSubsystem, SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        this.spindexerSubsystem = spindexerSubsystem;
        this.kickSubsystem = kickSubsystem;
        addRequirements(intakeSubsystem, spindexerSubsystem, kickSubsystem);
    }

    @Override
    public void initialize() {
        slotCmd = new SlotCmd(spindexerSubsystem, kickSubsystem, spindexerSubsystem.getSlot()+1);
    }

    @Override
    public void execute(){
        if(!spindexerSubsystem.fullSpindexer() && !spindexerSubsystem.ballInBayOne()) {
            intakeSubsystem.activate(Constants.INTAKE_SPEED);
        } else {
            intakeSubsystem.stop();
        }

        if(spindexerSubsystem.ballInBayOne() && (!spindexerSubsystem.ballInBayThree() || !spindexerSubsystem.ballInBayTwo()) && !slotCmd.isScheduled()){
            slotCmd.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
