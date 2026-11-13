package org.firstinspires.ftc.team2844simple.commands.drive;

import com.pedropathing.math.Pose;
import com.vcs.valleylib.core.command.Command;

import org.firstinspires.ftc.team2844simple.subsystems.DriveSubsystem;

public class HoldPosition implements Command {
    public DriveSubsystem drive;
    Pose current;
    public HoldPosition(DriveSubsystem drive){
        this.drive = drive;
    }

    @Override
    public void initialize() {
        current = drive.getPose();
    }

    @Override
    public void execute() {
        drive.holdPosition(current);
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) drive.stop();
    }
}
