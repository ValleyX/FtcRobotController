package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

public class AimToLLCmd extends CommandBase {
    AimSubsystem aimSubsystem;
    SensorSubsystem sensorSubsystem;

    public AimToLLCmd(AimSubsystem aimSubsystem, SensorSubsystem sensorSubsystem){
        this.aimSubsystem = aimSubsystem;
        this.sensorSubsystem = sensorSubsystem;
        addRequirements(aimSubsystem, sensorSubsystem);
    }

    @Override
    public void execute(){
        double tx = sensorSubsystem.getTx();
        //if(tx != Constants.NO_LL && (Math.abs(tx) < Constants.TURRET_THRESHHOLD)){
//        if(tx != Constants.NO_LL && (Math.abs(tx) < Constants.MAX_DEGREE)){
//            aimSubsystem.moveTurret(sensorSubsystem.getTx());
//        }
//        else {
//            aimSubsystem.moveTurret(0);
//        }

        //JAE
        //aimSubsystem.setPosLL(sensorSubsystem.getTx());

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
