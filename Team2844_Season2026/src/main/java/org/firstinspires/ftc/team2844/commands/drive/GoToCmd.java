package org.firstinspires.ftc.team2844.commands.drive;

import com.pedropathing.paths.Path;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.subsystem.Subsystem;
import com.vcs.valleylib.ftc.pedro.PedroSubsystem;

import java.util.Collections;
import java.util.Set;
import java.util.function.Supplier;

public class GoToCmd implements Command {
    private final PedroSubsystem drive;
    private final Supplier<Path> path;
    private final double maxSpeed;

    public GoToCmd(PedroSubsystem drive, Supplier<Path> path, double maxSpeed){
        this.drive = drive;
        this.path = path;
        this.maxSpeed = maxSpeed;
    }

    public GoToCmd(PedroSubsystem drive, Supplier<Path> path){
        this.drive = drive;
        this.path = path;
        this.maxSpeed = 1.0;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(drive);
    }

    @Override
    public void initialize() {
        Path p = path.get();
        drive.getFollower().follow(drive.withSpeedLimit(p, maxSpeed));
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
