package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import static org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants.TURN_TICK;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

public class MoveTurretToPos extends CommandBase {
    AimSubsystem aimSubsystem;
    double setDeg;

    public MoveTurretToPos(AimSubsystem aimSubsystem, double servoDeg){
        this.aimSubsystem = aimSubsystem;
        setDeg = servoDeg;
    }

    @Override
    public void initialize(){
        aimSubsystem.moveTurret(setDeg);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
