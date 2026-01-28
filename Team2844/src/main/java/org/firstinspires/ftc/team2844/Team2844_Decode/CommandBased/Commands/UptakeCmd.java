package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;

public class UptakeCmd extends CommandBase {
    KickSubsystem kickSubsystem;

    public UptakeCmd(KickSubsystem kickSubsystem){
        this.kickSubsystem = kickSubsystem;
    }

    @Override
    public void initialize() {
        kickSubsystem.rotateKickerDown();
    }

    @Override
    public void execute() {
        kickSubsystem.runKickerSpin();
        kickSubsystem.runSFeedForward();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
