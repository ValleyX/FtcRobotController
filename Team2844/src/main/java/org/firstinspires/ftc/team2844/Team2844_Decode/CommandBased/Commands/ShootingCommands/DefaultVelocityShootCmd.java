package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Subsystems;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultVelocityShootCmd extends CommandBase {
    DriveSubsystem driveSubsystem;
    ShooterSubsystem shooterSubsystem;
    DoubleSupplier botX;
    DoubleSupplier botY;
    BooleanSupplier full;
    int pipeline;

    public DefaultVelocityShootCmd(ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem, int pipeline, DoubleSupplier botX, DoubleSupplier botY, BooleanSupplier full){
        this.driveSubsystem = driveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.pipeline = pipeline;

        this.botX = botX;
        this.botY = botY;
        this.full = full;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        double botXValue = botX.getAsDouble();
        double botYValue = botY.getAsDouble();
        if(botXValue < 0.0 && botYValue < 0.0){
            if(botXValue > botYValue){
                if(full.getAsBoolean())
                    shooterSubsystem.setVelocity(driveSubsystem.velocityLinReg(pipeline));
            }
        } else if(botXValue < 0.0 && botYValue > 0.0){
            if(botYValue < -botXValue){
                if(full.getAsBoolean())
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
