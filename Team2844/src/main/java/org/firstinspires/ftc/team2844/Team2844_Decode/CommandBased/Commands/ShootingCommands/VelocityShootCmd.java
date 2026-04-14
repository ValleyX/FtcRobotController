package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class VelocityShootCmd extends CommandBase {
    ShooterSubsystem shooterSubsystem;
    DoubleSupplier velocity;

    public VelocityShootCmd(ShooterSubsystem shooterSubsystem, DoubleSupplier velocity){
        this.velocity = velocity;
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setVelocity(velocity.getAsDouble());;
    }

    @Override
    public boolean isFinished() {
        //return shooterSubsystem.inRange(velocity, () -> shooterSubsystem.getVelocity()).getAsBoolean();
        return true;
    }
}
