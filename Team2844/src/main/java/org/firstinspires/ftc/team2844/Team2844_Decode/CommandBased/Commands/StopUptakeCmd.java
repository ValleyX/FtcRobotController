package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;

public class StopUptakeCmd extends CommandBase {
    KickSubsystem kickSubsystem;

    public StopUptakeCmd(KickSubsystem kickSubsystem) {
        this.kickSubsystem = kickSubsystem;
    }

    @Override
    public void initialize() {
        kickSubsystem.rotateKickerUp();
        kickSubsystem.stopSFeed();
        kickSubsystem.stopKickerSpin();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
