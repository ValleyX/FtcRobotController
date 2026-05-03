package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

public class DefaultAimCmd extends CommandBase {
    AimSubsystem aimSubsystem;
    DriveSubsystem driveSubsystem;
    SensorSubsystem sensorSubsystem;
    int pipeline;

    public DefaultAimCmd(AimSubsystem aimSubsystem, DriveSubsystem driveSubsystem, SensorSubsystem sensorSubsystem, int pipeline){
        this.aimSubsystem = aimSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.sensorSubsystem = sensorSubsystem;
        this.pipeline = pipeline;
        addRequirements(aimSubsystem);
    }

    @Override
    public void execute() {
        double targetAngle = driveSubsystem.getPinpointTurretAngle(pipeline);
        double tx = sensorSubsystem.getTx();
        if(tx != Constants.NO_LL){
            if(!(Math.abs(tx) < Constants.TURRET_THRESHHOLD) && (Math.abs(tx) < Constants.MAX_NEUTRAL)) {
                double pos = aimSubsystem.getAxonValue();

                //aimSubsystem.aimTurret(pos - tx);
                if(tx < 0.0){
                    aimSubsystem.aimTurret(pos + Math.min(8.0, Math.abs(tx)));
                } else if (tx > 0.0){
                    aimSubsystem.aimTurret(pos - Math.min(8.0, tx));
                }
            }
        } else if(Math.abs(targetAngle-Constants.NEUTRAL_TURRET) < Constants.MAX_NEUTRAL){
            aimSubsystem.aimTurret(targetAngle);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
