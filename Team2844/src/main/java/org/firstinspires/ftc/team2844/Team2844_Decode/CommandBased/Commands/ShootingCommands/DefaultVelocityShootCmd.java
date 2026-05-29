package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Subsystems;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultVelocityShootCmd extends CommandBase {
    ShooterSubsystem shooterSubsystem;
    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    ShooterFeedSubsystem shooterFeedSubsystem;
    int pipeline;

    public DefaultVelocityShootCmd(Subsystems subsystems){
        this.driveSubsystem = subsystems.mecDriveSubsystem;
        this.shooterSubsystem = subsystems.shooterSubsystem;
        this.intakeSubsystem = subsystems.intakeSubsystem;
        this.shooterFeedSubsystem = subsystems.shooterFeedSubsystem;
        this.pipeline = subsystems.sensorSubsystem.getPipeline();

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        double botX = driveSubsystem.getBotX();
        double botY = driveSubsystem.getBotY();
        if(botX < 0.0 && botY < 0.0){
            if(botX > botY){
                if(shooterFeedSubsystem.topBroken() && intakeSubsystem.ballInBeam())
                    shooterSubsystem.setVelocity(driveSubsystem.velocityLinReg(pipeline));
            }
        } else if(botX < 0.0 && botY > 0.0){
            if(botY < -botX){
                if(shooterFeedSubsystem.topBroken() && intakeSubsystem.ballInBeam())
                    shooterSubsystem.setVelocity(driveSubsystem.velocityLinReg(pipeline));
            }
        } else {
            shooterSubsystem.setVelocity(1000);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
