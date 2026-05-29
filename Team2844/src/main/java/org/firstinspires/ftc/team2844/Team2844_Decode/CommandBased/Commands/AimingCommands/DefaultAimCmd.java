package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

import java.util.function.BooleanSupplier;

public class DefaultAimCmd extends FullAimToLLCmd {

    AimSubsystem aimSubsystem;
    DriveSubsystem driveSubsystem;
    SensorSubsystem sensorSubsystem;
    BooleanSupplier manualAim;
    boolean init;
    public DefaultAimCmd(AimSubsystem aimSubsystem, DriveSubsystem driveSubsystem, SensorSubsystem sensorSubsystem, BooleanSupplier manualAim){
        super(aimSubsystem, sensorSubsystem, driveSubsystem);
        this.aimSubsystem = aimSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.sensorSubsystem = sensorSubsystem;
        this.manualAim = manualAim;
        init = true;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        if(!manualAim.getAsBoolean()){
            double botX = driveSubsystem.getBotX();
            double botY = driveSubsystem.getBotY();
            double turretAngle = driveSubsystem.getPinpointTurretAngle(sensorSubsystem.getPipeline());
            if(botX < 0.0 && botY < 0.0){
                if(botX > botY){
                    if(init){
                        init = false;
                        super.initialize();
                    }
                    if(Constants.MIN_DEGREE + 10 < turretAngle
                            && turretAngle < Constants.MAX_DEGREE - 10)
                        super.execute();
                }
            } else if(botX < 0.0 && botY > 0.0){
                if(botY < -botX){
                    if(init){
                        init = false;
                        super.initialize();
                    }
                    if(Constants.MIN_DEGREE + 10 < turretAngle
                            && turretAngle < Constants.MAX_DEGREE - 10)
                        super.execute();
                }
            } else {
                init = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
