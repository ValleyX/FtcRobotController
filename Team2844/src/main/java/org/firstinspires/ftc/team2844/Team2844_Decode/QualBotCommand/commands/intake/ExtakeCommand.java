package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.intake;

import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.subsystem.Subsystem;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems.IntakeSubsystem;

import java.util.Collections;
import java.util.Set;
import java.util.function.DoubleSupplier;

/** Runs the intake backwards to spit a jammed or unwanted ball back out. */
public class ExtakeCommand implements Command {

    private final IntakeSubsystem intake;
    private final DoubleSupplier power;

    public ExtakeCommand(IntakeSubsystem intake, DoubleSupplier power) {
        this.intake = intake;
        this.power = power;
    }

    @Override
    public void execute() {
        intake.extake(power.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(intake);
    }

    @Override
    public String getName() {
        return "Extake";
    }
}
