package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;

public class FullUptakeCmd extends CommandBase {
    KickSubsystem kickSubsystem;
    ShooterFeedSubsystem shooterFeedSubsystem;
    public FullUptakeCmd(KickSubsystem kickSubsystem, ShooterFeedSubsystem shooterFeedSubsystem){
        this.kickSubsystem = kickSubsystem;
        this.shooterFeedSubsystem = shooterFeedSubsystem;
    }

    @Override
    public void execute() {
        kickSubsystem.runKickerSpin();
        kickSubsystem.runSFeedForward();

        if(shooterFeedSubsystem.topBroken()){
            kickSubsystem.rotateKickerDownExtra();
        } else {
            kickSubsystem.rotateKickerDown();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
