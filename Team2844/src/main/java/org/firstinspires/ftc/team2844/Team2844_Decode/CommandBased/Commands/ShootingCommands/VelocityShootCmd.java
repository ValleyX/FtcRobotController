package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;

public class VelocityShootCmd extends CommandBase {
    ShooterSubsystem shooterSubsystem;
    double velocity;
    public VelocityShootCmd(ShooterSubsystem shooterSubsystem, double velocity){
        this.velocity = velocity;
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setVelocity(velocity);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
