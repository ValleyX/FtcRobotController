package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.LimelightSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

public class AimToLLCmd extends CommandBase {
    AimSubsystem aimSubsystem;
    LimelightSubsystem limelightSubsystem;

    public AimToLLCmd(AimSubsystem aimSubsystem, LimelightSubsystem limelightSubsystem){
        this.aimSubsystem = aimSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(aimSubsystem, limelightSubsystem);
    }

    @Override
    public void execute(){
        double tx = limelightSubsystem.getTx();
        if(tx != Constants.NO_LL && (Math.abs(tx) < Constants.TURRET_THRESHHOLD)){
            aimSubsystem.moveTurret(limelightSubsystem.getTx());
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
