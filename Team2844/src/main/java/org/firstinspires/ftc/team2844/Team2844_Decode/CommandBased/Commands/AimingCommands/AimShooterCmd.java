package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;/*
package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.LimelightSubsystem;

public class AimShooterCmd extends ParallelCommandGroup {
    AimSubsystem aimSubsystem;
    LimelightSubsystem limelightSubsystem;

    public AimShooterCmd(AimSubsystem aimSubsystem, LimelightSubsystem limelightSubsystem){
        this.aimSubsystem = aimSubsystem;
        this.limelightSubsystem = limelightSubsystem;

        addRequirements(aimSubsystem, limelightSubsystem);
    }

    @Override
    public void execute() {
        new ParallelCommandGroup(new AimHoodCmd(aimSubsystem, limelightSubsystem), new AimToLLCmd(aimSubsystem, limelightSubsystem)).schedule();
    }

    public boolean isFinished(){
        return false;
    }
}
*/
