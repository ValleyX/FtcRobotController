package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultVelocityRegCmd extends CommandBase {
    ShooterSubsystem shooterSubsystem;
    DriveSubsystem driveSubsystem;
    int pipeline;
    DoubleSupplier velocity;

    public DefaultVelocityRegCmd(ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem, int pipeline, DoubleSupplier velocity){
        this.driveSubsystem = driveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.pipeline = pipeline;
        this.velocity = velocity;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.setVelocity(velocity.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        //return shooterSubsystem.inRange(velocity, () -> shooterSubsystem.getVelocity()).getAsBoolean();
        return false;
    }
}
