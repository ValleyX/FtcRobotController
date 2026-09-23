package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;

public class StopSpinCmd extends CommandBase {
    KickSubsystem kickSubsystem;
    public StopSpinCmd(KickSubsystem kickSubsystem){
        this.kickSubsystem = kickSubsystem;
    }

    @Override
    public void initialize() {
        kickSubsystem.stopSFeed();
        kickSubsystem.stopKickerSpin();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
