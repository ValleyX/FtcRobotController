package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import static org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Constants.TURN_TICK;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

public class MoveTurretNegative extends CommandBase {
    AimSubsystem aimSubsystem;

    public MoveTurretNegative(AimSubsystem aimSubsystem){
        this.aimSubsystem = aimSubsystem;
    }

    @Override
    public void initialize(){
        aimSubsystem.moveTurret(-TURN_TICK);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
