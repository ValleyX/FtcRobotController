package org.firstinspires.ftc.team2844.commands.drive;

import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.subsystem.Subsystem;

import org.firstinspires.ftc.team2844.subsystems.DriveSubsystem;

import java.util.Collections;
import java.util.Set;

public class TurnToCmd implements Command {
    private final DriveSubsystem drive;
    private final double radians;
    public TurnToCmd(DriveSubsystem drive, double radians){
        this.drive = drive;
        this.radians = radians;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(drive);
    }

    @Override
    public void initialize() {
        drive.turnTo(radians);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return drive.isIdle();
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) drive.stop();
    }
}
