package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.PinpointSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.LimelightSubsystem;

public class FullAimToLLCmd extends CommandBase {
    AimSubsystem aimSubsystem;
    LimelightSubsystem limelightSubsystem;
    //ElapsedTime elapsedTime;
    //double time;
    //double initTime;
    double tx;
    double desiredPos;
    //boolean waitTilBusy;


    public FullAimToLLCmd(AimSubsystem aimSubsystem, LimelightSubsystem limelightSubsystem){
        this.aimSubsystem = aimSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(aimSubsystem, limelightSubsystem);
        //elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        //waitTilBusy = false;
        //elapsedTime.reset();
    }


    @Override
    public void initialize(){
        tx = limelightSubsystem.getTx();

        if(tx != Constants.NO_LL){
            if(!(Math.abs(tx) < Constants.TURRET_THRESHHOLD)) {
                double pos = aimSubsystem.getEncoderDegrees();
                desiredPos = ((pos + tx)+((pos-Constants.TURRET_OFFSET)*0.2));
                //desiredPos = (aimSubsystem.getTurretServoDegrees() + tx);
                aimSubsystem.aimTurret(desiredPos);
            }
            aimSubsystem.aimHood(limelightSubsystem.hoodLinReg());
        }
    }

    /*@Override
    public void execute(){

    }*/
/*
    public void end(){
        new FullAimToLLCmd(aimSubsystem, limelightSubsystem);
    }
    */
    @Override
    public boolean isFinished() {
        /*if((Math.abs(tx) <= Constants.TURRET_THRESHHOLD)){
            return true;
        } else {
            return false;
        }
*/
        return true;
    }
}
