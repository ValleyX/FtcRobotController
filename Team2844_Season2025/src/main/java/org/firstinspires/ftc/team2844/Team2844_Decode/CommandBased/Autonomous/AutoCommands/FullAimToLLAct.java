package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

import java.util.concurrent.TimeUnit;

public class FullAimToLLAct implements Action {
    AimSubsystem aimSubsystem;
    SensorSubsystem sensorSubsystem;
    DriveSubsystem driveSubsystem;
    double tx;
    double pos;
    double savedTx;
    boolean seen;
    boolean looped;
    ElapsedTime llTimer;
    ElapsedTime ppTimer;
    boolean finished = true;
    boolean const2 = false;
    boolean init;

    public FullAimToLLAct(AimSubsystem aimSubsystem, SensorSubsystem sensorSubsystem, DriveSubsystem driveSubsystem){
        this.aimSubsystem = aimSubsystem;
        this.sensorSubsystem = sensorSubsystem;
        this.driveSubsystem = driveSubsystem;
        llTimer = new ElapsedTime();
        ppTimer = new ElapsedTime();
        const2 = false;
        init = true;
    }

    public FullAimToLLAct(AimSubsystem aimSubsystem, SensorSubsystem sensorSubsystem, DriveSubsystem driveSubsystem, boolean finished){
        this.aimSubsystem = aimSubsystem;
        this.sensorSubsystem = sensorSubsystem;
        this.driveSubsystem = driveSubsystem;
        llTimer = new ElapsedTime();
        ppTimer = new ElapsedTime();
        this.finished = finished;
        const2 = true;
        init = true;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if(init){
            savedTx = 0;
            llTimer.reset();
            ppTimer.reset();
            seen = false;
            looped = false;
            init = false;
        }


        tx = sensorSubsystem.getTx();
        pos = aimSubsystem.getAxonValue();

        if(tx == Constants.NO_LL){
            ppTimer.reset();
        }

        if (tx != Constants.NO_LL && (ppTimer.time(TimeUnit.MILLISECONDS) > 100 || !looped)) {
            llTimer.reset();
            seen = true;
            if(ppTimer.time(TimeUnit.MILLISECONDS) > 100)
                looped = true;
            if (!(Math.abs(tx) < Constants.TURRET_THRESHHOLD)) {
                savedTx = tx;
                //aimSubsystem.aimTurret(pos - tx);
                if (tx < 0.0) {
                    aimSubsystem.aimTurret(pos + Math.min(8.0, Math.abs(tx)));
                } else if (tx > 0.0) {
                    aimSubsystem.aimTurret(pos - Math.min(8.0, Math.abs(tx)));
                }
            }
            aimSubsystem.aimHood(driveSubsystem.hoodLinReg(sensorSubsystem.getPipeline()));

        } else if(llTimer.time(TimeUnit.MILLISECONDS) < 100 && seen) {
            if (!(Math.abs(savedTx) < Constants.TURRET_THRESHHOLD)) {
                if (savedTx < 0.0) {
                    aimSubsystem.aimTurret(pos + Math.min(8.0, Math.abs(savedTx)));
                } else if (savedTx > 0.0) {
                    aimSubsystem.aimTurret(pos - Math.min(8.0, Math.abs(savedTx)));
                }
            }
            aimSubsystem.aimHood(driveSubsystem.hoodLinReg(sensorSubsystem.getPipeline()));
        }else {
            aimSubsystem.aimTurret(driveSubsystem.getPinpointTurretAngle(sensorSubsystem.getPipeline()));
        }

        if(const2){
            return !finished;
        }
        return !((Math.abs(tx) < Constants.TURRET_THRESHHOLD)) &&
                !( Math.abs(driveSubsystem.getPinpointTurretAngle(sensorSubsystem.getPipeline()) - aimSubsystem.getTurretDegrees()) < Constants.TURRET_THRESHHOLD);
    }
}
