package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

import java.util.concurrent.TimeUnit;

public class FullAimToLLCmd extends CommandBase {
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

    public FullAimToLLCmd(AimSubsystem aimSubsystem, SensorSubsystem sensorSubsystem, DriveSubsystem driveSubsystem){
        this.aimSubsystem = aimSubsystem;
        this.sensorSubsystem = sensorSubsystem;
        this.driveSubsystem = driveSubsystem;
        llTimer = new ElapsedTime();
        ppTimer = new ElapsedTime();
        addRequirements(aimSubsystem);
    }


    @Override
    public void initialize(){
        savedTx = 0;
        llTimer.reset();
        ppTimer.reset();
        seen = false;
        looped = false;
    }

    public void execute() {
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

            //if you are close to the apriltag, reset your position. It is more accurate when it is close.
            //this makes the values freak out. probably cause the limelight is updating the pinpoint's pose but then the pinpoint is updating the limelight's orientation at the same time
            //if(sensorSubsystem.getDis() < 50.0){
            //driveSubsystem.setPinpointPose(new Pose2d(sensorSubsystem.getBotXLLMT2(), sensorSubsystem.getBotYLLMT2(), driveSubsystem.getRobotHeading()));
            //}
            //driveSubsystem.setPinpointPose();
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
    }


    @Override
    public boolean isFinished() {
        return ((Math.abs(tx) < Constants.TURRET_THRESHHOLD)) ||
                ( Math.abs(driveSubsystem.getPinpointTurretAngle(sensorSubsystem.getPipeline()) - aimSubsystem.getTurretDegrees()) < Constants.TURRET_THRESHHOLD);
    }
}
