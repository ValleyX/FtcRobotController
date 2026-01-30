package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

public class IntakeCmd extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private SpindexerSubsystem spindexerSubsystem;

    public IntakeCmd(IntakeSubsystem intakeSubsystem, SpindexerSubsystem spindexerSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        this.spindexerSubsystem = spindexerSubsystem;
        addRequirements(intakeSubsystem, spindexerSubsystem);
    }

    @Override
    public void execute(){
        intakeSubsystem.activate(0.9);

        if(spindexerSubsystem.ballInBayOne() && !spindexerSubsystem.ballInBayThree()){
            spindexerSubsystem.nextBay();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
