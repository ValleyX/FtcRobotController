package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.FullAimToLLCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;

public class SmartShooterCmd extends CommandBase {

    ShooterSubsystem shooterSubsystem;
    SensorSubsystem sensorSubsystem;

    AimSubsystem aimSubsystem;

    public SmartShooterCmd(ShooterSubsystem shooterSubsystem, SensorSubsystem sensorSubsystem, AimSubsystem aimSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        this.sensorSubsystem = sensorSubsystem;
        this.aimSubsystem = aimSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute(){
        new FullAimToLLCmd(aimSubsystem, sensorSubsystem);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
