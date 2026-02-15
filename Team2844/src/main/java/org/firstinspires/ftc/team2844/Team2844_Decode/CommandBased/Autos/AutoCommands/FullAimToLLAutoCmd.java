package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands;

import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

public class FullAimToLLAutoCmd extends CommandBase {
    AimSubsystem aimSubsystem;
    SensorSubsystem sensorSubsystem;
    double tx;
    boolean finished;
    Vector2d vector;
    double heading;


    public FullAimToLLAutoCmd(AimSubsystem aimSubsystem, SensorSubsystem sensorSubsystem, Vector2d vector, double heading){
        this.aimSubsystem = aimSubsystem;
        this.sensorSubsystem = sensorSubsystem;
        this.vector = vector;
        this.heading = heading;
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
            aimSubsystem.aimTurret(sensorSubsystem.getPinpointTurretAngleAuto(vector.x, vector.y, heading));
        }

    }


    @Override
    public boolean isFinished() {
        return finished;
    }
}
