package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.LimelightSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

public class AimTurretCmd extends CommandBase {
    AimSubsystem aimSubsystem;
    LimelightSubsystem limelightSubsystem;

    public AimTurretCmd(AimSubsystem aimSubsystem, LimelightSubsystem limelightSubsystem){
        this.aimSubsystem = aimSubsystem;
        this.limelightSubsystem = limelightSubsystem;

        addRequirements(aimSubsystem, limelightSubsystem);
    }

    public void execute(){
        aimSubsystem.aimTurret(0.5);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
