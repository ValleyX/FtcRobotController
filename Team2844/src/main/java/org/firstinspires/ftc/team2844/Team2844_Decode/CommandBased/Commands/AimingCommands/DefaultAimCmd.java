package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultAimCmd extends CommandBase {

    AimSubsystem aimSubsystem;
    DriveSubsystem driveSubsystem;
    SensorSubsystem sensorSubsystem;
    DoubleSupplier botX;
    DoubleSupplier botY;
    BooleanSupplier manualAim;
    boolean init;
    public DefaultAimCmd(AimSubsystem aimSubsystem, DriveSubsystem driveSubsystem, DoubleSupplier botX, DoubleSupplier botY, BooleanSupplier manualAim){
        this.aimSubsystem = aimSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.manualAim = manualAim;
        this.botX = botX;
        this.botY = botY;
        init = true;
    }

    @Override
    public void execute() {
        if(!manualAim.getAsBoolean()){
            double botXValue = botX.getAsDouble();
            double botYValue = botY.getAsDouble();
            double turretAngle = driveSubsystem.getPinpointTurretAngle(sensorSubsystem.getPipeline());
            if(botXValue < 0.0 && botYValue < 0.0){
                if(botXValue > botYValue){
                    if(Constants.MIN_DEGREE + 10 < turretAngle
                            && turretAngle < Constants.MAX_DEGREE - 10)
                        aimSubsystem.aimTurret(turretAngle);
                }
            } else if(botXValue < 0.0 && botYValue > 0.0){
                if(botYValue < -botXValue){
                    if(Constants.MIN_DEGREE + 10 < turretAngle
                            && turretAngle < Constants.MAX_DEGREE - 10)
                        aimSubsystem.aimTurret(turretAngle);
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
