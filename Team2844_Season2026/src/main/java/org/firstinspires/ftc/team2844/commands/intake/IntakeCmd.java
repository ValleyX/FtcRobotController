package org.firstinspires.ftc.team2844.commands.intake;

import com.vcs.valleylib.core.command.Command;

import org.firstinspires.ftc.team2844.subsystems.IntakeSubsystem;

public class IntakeCmd implements Command {
    IntakeSubsystem intake;
    public IntakeCmd(IntakeSubsystem intake){
        this.intake = intake;
    }

    @Override
    public void execute() {
        intake.intake();
    }

    @Override
    public boolean isFinished() {
        return intake.full();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }
}
