package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;

public class StopIntakeCmd  extends CommandBase {
    private IntakeSubsystem intakeSubsystem;

    public StopIntakeCmd(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute(){
        intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
