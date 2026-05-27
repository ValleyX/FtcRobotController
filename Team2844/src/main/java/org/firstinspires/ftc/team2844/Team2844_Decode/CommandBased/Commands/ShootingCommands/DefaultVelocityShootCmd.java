package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultVelocityShootCmd extends CommandBase {
    ShooterSubsystem shooterSubsystem;
    DriveSubsystem driveSubsystem;
    int pipeline;

    public DefaultVelocityShootCmd(ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem, int pipeline){
        this.driveSubsystem = driveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.pipeline = pipeline;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        //shooterSubsystem.setVelocity(driveSubsystem.velocityLinReg(pipeline));
        shooterSubsystem.setVelocity(1000);
        //shooterSubsystem.setVelocity(shooterSubsystem.getVelocity());
    }

    @Override
    public boolean isFinished() {
        //return shooterSubsystem.inRange(velocity, () -> shooterSubsystem.getVelocity()).getAsBoolean();
        return false;
    }
}
