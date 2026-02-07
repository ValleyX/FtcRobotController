package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.NextSlotCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

public class IntakeCmd extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private SpindexerSubsystem spindexerSubsystem;
    private KickSubsystem kickSubsystem;
    private boolean passed;


    public IntakeCmd(IntakeSubsystem intakeSubsystem, SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        this.spindexerSubsystem = spindexerSubsystem;
        this.kickSubsystem = kickSubsystem;
        addRequirements(intakeSubsystem, spindexerSubsystem);
        passed = false;
    }

    @Override
    public void execute(){
        if(!spindexerSubsystem.full()) {
            intakeSubsystem.activate(Constants.INTAKE_SPEED);
        }

        if(intakeSubsystem.ballInBeam()){
            passed = true;
        }

        if((!spindexerSubsystem.ballInBayThree() || !spindexerSubsystem.ballInBayTwo())){

            new NextSlotCmd(spindexerSubsystem, kickSubsystem);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
