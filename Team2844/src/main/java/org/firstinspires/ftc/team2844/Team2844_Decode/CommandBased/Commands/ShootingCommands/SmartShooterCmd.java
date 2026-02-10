package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.LimelightSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;

public class SmartShooterCmd extends CommandBase {

    ShooterSubsystem shooterSubsystem;
    LimelightSubsystem limelightSubsystem;

    AimSubsystem aimSubsystem;

    public SmartShooterCmd(ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem, AimSubsystem aimSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.aimSubsystem = aimSubsystem;

        addRequirements(shooterSubsystem, aimSubsystem);
    }

    @Override
    public void execute(){


    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
