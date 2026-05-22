package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

public class AimTurretCmd extends CommandBase {
    AimSubsystem aimSubsystem;
    double degrees;

    public AimTurretCmd(AimSubsystem aimSubsystem, double degrees){
        this.aimSubsystem = aimSubsystem;
        this.degrees = degrees;
    }

    @Override
    public void initialize() {
        aimSubsystem.aimTurret(degrees);
    }

    @Override
    public boolean isFinished() {
        return (degrees - Constants.TURRET_THRESHHOLD <= aimSubsystem.getTurretDegrees() && aimSubsystem.getTurretDegrees() <= degrees + Constants.TURRET_THRESHHOLD);
    }
}
