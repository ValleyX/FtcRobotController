package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;

public class IntakeCmd extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private double power;

    public IntakeCmd(IntakeSubsystem intakeSubsystem, double power){
        this.intakeSubsystem = intakeSubsystem;
        this.power = power;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute(){
        intakeSubsystem.activate(power);
    }

    @Override
    public boolean isFinished() {
        return !(power > 0);
    }
}
