package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

public class FullAimToLLAutoCmd extends CommandBase {
    AimSubsystem aimSubsystem;
    SensorSubsystem sensorSubsystem;
    double tx;
    boolean finished;


    public FullAimToLLAutoCmd(AimSubsystem aimSubsystem, SensorSubsystem sensorSubsystem){
        this.aimSubsystem = aimSubsystem;
        this.sensorSubsystem = sensorSubsystem;
        addRequirements(aimSubsystem);
    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute(){
        tx = sensorSubsystem.getTx();

        if(tx != Constants.NO_LL){
            if(!(Math.abs(tx) < Constants.TURRET_THRESHHOLD)) {
                double pos = aimSubsystem.getTurretDegrees();
                aimSubsystem.aimTurret(pos + tx);
            } else {
                finished = true;
            }
            aimSubsystem.aimHood(sensorSubsystem.hoodLinReg());
        } else {
            aimSubsystem.aimTurret(sensorSubsystem.getPinpointTurretAngle());
        }

    }


    @Override
    public boolean isFinished() {
        return finished;
    }
}
