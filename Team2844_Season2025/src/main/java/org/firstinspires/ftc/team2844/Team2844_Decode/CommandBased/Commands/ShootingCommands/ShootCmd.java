package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;

public class ShootCmd extends CommandBase {

    ShooterSubsystem shooterSubsystem;

    public ShootCmd(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute(){
        shooterSubsystem.setPower(0.5);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

