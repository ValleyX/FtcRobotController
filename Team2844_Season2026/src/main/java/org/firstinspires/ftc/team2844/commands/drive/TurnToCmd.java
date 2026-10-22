package org.firstinspires.ftc.team2844.commands.drive;

import com.vcs.valleylib.core.command.Command;

import org.firstinspires.ftc.team2844.subsystems.DriveSubsystem;

public class TurnToCmd implements Command {
    private final DriveSubsystem drive;
    private final double radians;
    public TurnToCmd(DriveSubsystem drive, double radians){
        this.drive = drive;
        this.radians = radians;
    }

    @Override
    public void initialize() {
        Command.super.initialize();
    }

    @Override
    public void execute() {
        drive.turnTo(radians);
    }

    @Override
    public boolean isFinished() {
        return drive.getFollower().idle();
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) drive.stop();
    }
}
