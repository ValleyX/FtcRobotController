package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

public class FullAimToLLCmd extends CommandBase {
    AimSubsystem aimSubsystem;
    SensorSubsystem sensorSubsystem;
    //ElapsedTime elapsedTime;
    //double time;
    //double initTime;
    double tx;
    double desiredPos;
    //boolean waitTilBusy;


    public FullAimToLLCmd(AimSubsystem aimSubsystem, SensorSubsystem sensorSubsystem){
        this.aimSubsystem = aimSubsystem;
        this.sensorSubsystem = sensorSubsystem;
        addRequirements(aimSubsystem);
        //elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        //waitTilBusy = false;
        //elapsedTime.reset();
    }


    @Override
    public void initialize(){
        tx = sensorSubsystem.getTx();

        if(tx != Constants.NO_LL){
            if(!(Math.abs(tx) < Constants.TURRET_THRESHHOLD)) {
                double pos = aimSubsystem.getTurretDegrees();
                aimSubsystem.setPosition(pos + tx);
            }
            aimSubsystem.aimHood(sensorSubsystem.hoodLinReg());
        }
    }

    /*@Override
    public void execute(){

    }*/
/*
    public void end(){
        new FullAimToLLCmd(aimSubsystem, limelightSubsystem);
    }
    */
    @Override
    public boolean isFinished() {
        /*if((Math.abs(tx) <= Constants.TURRET_THRESHHOLD)){
            return true;
        } else {
            return false;
        }
*/
        return true;
    }
}
