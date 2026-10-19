package org.firstinspires.ftc.team2844.commands.intake;

import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.subsystem.Subsystem;

import org.firstinspires.ftc.team2844.subsystems.IntakeSubsystem;

import java.util.Collections;
import java.util.Set;

public class IntakeCmd implements Command {
    IntakeSubsystem intake;
    public IntakeCmd(IntakeSubsystem intake){
        this.intake = intake;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(intake);
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
