package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.SlotCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

public class IntakeSortCmd extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private SpindexerSubsystem spindexerSubsystem;
    private KickSubsystem kickSubsystem;
    private boolean passed;


    public IntakeSortCmd(IntakeSubsystem intakeSubsystem, SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        this.spindexerSubsystem = spindexerSubsystem;
        this.kickSubsystem = kickSubsystem;
        addRequirements(intakeSubsystem);
        passed = true;
    }

    @Override
    public void execute(){
        if(!spindexerSubsystem.fullSpindexer()) {
            intakeSubsystem.activate(Constants.INTAKE_SPEED);
        } else {
            intakeSubsystem.stop();
        }

        if(spindexerSubsystem.ballInBayOne() && (!spindexerSubsystem.ballInBayThree() || !spindexerSubsystem.ballInBayTwo())){
            new SlotCmd(spindexerSubsystem, kickSubsystem, spindexerSubsystem.getSlot()+1);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
