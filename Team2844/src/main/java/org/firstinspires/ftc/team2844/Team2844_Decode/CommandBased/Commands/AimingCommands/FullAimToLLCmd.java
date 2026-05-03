package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

public class FullAimToLLCmd extends CommandBase {
    AimSubsystem aimSubsystem;
    SensorSubsystem sensorSubsystem;
    DriveSubsystem driveSubsystem;
    double tx;


    public FullAimToLLCmd(AimSubsystem aimSubsystem, SensorSubsystem sensorSubsystem, DriveSubsystem driveSubsystem){
        this.aimSubsystem = aimSubsystem;
        this.sensorSubsystem = sensorSubsystem;
        this.driveSubsystem = driveSubsystem;
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
            aimSubsystem.aimHood(driveSubsystem.hoodLinReg(sensorSubsystem.getPipeline()));

            //if you are close to the apriltag, reset your position. It is more accurate when it is close.
            //this makes the values freak out. probably cause the limelight is updating the pinpoint's pose but then the pinpoint is updating the limelight's orientation at the same time
            //if(sensorSubsystem.getDis() < 50.0){
                //driveSubsystem.setPinpointPose(new Pose2d(sensorSubsystem.getBotXLLMT2(), sensorSubsystem.getBotYLLMT2(), driveSubsystem.getRobotHeading()));
            //}
            //driveSubsystem.setPinpointPose();
        } else {
            aimSubsystem.aimTurret(driveSubsystem.getPinpointTurretAngle(sensorSubsystem.getPipeline()));
            //aimSubsystem.aimTurret(Constants.NEUTRAL_TURRET);
        }
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}
