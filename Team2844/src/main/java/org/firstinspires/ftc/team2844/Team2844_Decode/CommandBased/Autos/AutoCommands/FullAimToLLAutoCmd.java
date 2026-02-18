package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands;

import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

public class FullAimToLLAutoCmd extends CommandBase {
    AimSubsystem aimSubsystem;
    SensorSubsystem sensorSubsystem;
    DriveSubsystem driveSubsystem;
    double tx;
    boolean finished;
    Vector2d vector;
    double heading;


    public FullAimToLLAutoCmd(AimSubsystem aimSubsystem, SensorSubsystem sensorSubsystem, DriveSubsystem driveSubsystem, Vector2d vector, double heading){
        this.aimSubsystem = aimSubsystem;
        this.sensorSubsystem = sensorSubsystem;
        this.driveSubsystem = driveSubsystem;
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
                //aimSubsystem.aimTurret(pos + tx);
                aimSubsystem.aimTurret(90.0);
            } else {
                finished = true;
            }
            aimSubsystem.aimHood(driveSubsystem.hoodLinReg(sensorSubsystem.getPipeline()));
        } else {
            //aimSubsystem.aimTurret(driveSubsystem.getPinpointTurretAngleAuto(vector.x, vector.y, heading, sensorSubsystem.getPipeline()));
            aimSubsystem.aimTurret(90.0);
        }

    }


    @Override
    public boolean isFinished() {
        return finished;
    }
}
