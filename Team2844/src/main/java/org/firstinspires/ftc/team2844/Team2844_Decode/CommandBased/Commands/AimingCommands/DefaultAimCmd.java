package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

public class DefaultAimCmd extends CommandBase {
    AimSubsystem aimSubsystem;
    DriveSubsystem driveSubsystem;
    int pipeline;

    public DefaultAimCmd(AimSubsystem aimSubsystem, DriveSubsystem driveSubsystem, int pipeline){
        this.aimSubsystem = aimSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.pipeline = pipeline;
        addRequirements(aimSubsystem);
    }

    @Override
    public void execute() {
        double targetAngle = driveSubsystem.getPinpointTurretAngle(pipeline);
        if(Math.abs(targetAngle-Constants.NEUTRAL_TURRET) < Constants.MAX_NEUTRAL){
            aimSubsystem.aimTurret(targetAngle);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
