package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

public class FullAimToLLCmd extends CommandBase {
    AimSubsystem aimSubsystem;
    SensorSubsystem sensorSubsystem;
    double tx;


    public FullAimToLLCmd(AimSubsystem aimSubsystem, SensorSubsystem sensorSubsystem){
        this.aimSubsystem = aimSubsystem;
        this.sensorSubsystem = sensorSubsystem;
        addRequirements(aimSubsystem);
    }


    @Override
    public void initialize(){
        tx = sensorSubsystem.getTx();

        if(tx != Constants.NO_LL){
            if(!(Math.abs(tx) < Constants.TURRET_THRESHHOLD)) {
                double pos = aimSubsystem.getAxonValue();

                //aimSubsystem.aimTurret(pos - tx);
                if(tx < 0.0){
                    aimSubsystem.aimTurret(pos + Math.min(8.0, Math.abs(tx)));
                } else if (tx > 0.0){
                    aimSubsystem.aimTurret(pos - Math.min(8.0, tx));
                }
            }
            aimSubsystem.aimHood(sensorSubsystem.hoodLinReg());
        } else {
            aimSubsystem.aimTurret(sensorSubsystem.getPinpointTurretAngle());
        }
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}
