package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;

public class UptakeExtraWaitCmd extends CommandBase {
    KickSubsystem kickSubsystem;
    ShooterFeedSubsystem shooterFeedSubsystem;

    public UptakeExtraWaitCmd(KickSubsystem kickSubsystem, ShooterFeedSubsystem shooterFeedSubsystem) {
        this.kickSubsystem = kickSubsystem;
        this.shooterFeedSubsystem = shooterFeedSubsystem;
        addRequirements(kickSubsystem);
    }

    @Override
    public void initialize() {
        kickSubsystem.rotateKickerDownExtra();
        kickSubsystem.runKickerSpin();
        kickSubsystem.runSFeedForward();
    }


    @Override
    public boolean isFinished() {
        return shooterFeedSubsystem.topBroken();
    }
}