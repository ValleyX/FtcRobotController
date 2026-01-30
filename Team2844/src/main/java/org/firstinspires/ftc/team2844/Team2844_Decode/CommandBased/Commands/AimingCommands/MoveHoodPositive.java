package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

public class MoveHoodPositive extends CommandBase {
    AimSubsystem aimSubsystem;

    public MoveHoodPositive(AimSubsystem aimSubsystem){
        this.aimSubsystem = aimSubsystem;
    }

    @Override
    public void initialize(){
        aimSubsystem.moveHood(-0.1);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
