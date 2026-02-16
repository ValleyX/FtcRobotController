package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ServoTuning;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;

public class KD_Tune extends CommandBase {
    AimSubsystem aimSubsystem;
    double KDIncrement;

    public KD_Tune(AimSubsystem aimSubsystem, double KD_Increment){
        this.aimSubsystem = aimSubsystem;
        KDIncrement = KD_Increment;
    }

    @Override
    public void initialize(){
        //reset the PID
       // aimSubsystem.servo.resetPID();
        //aimSubsystem.servo.setKD(Math.max(0, aimSubsystem.servo.getKD() + KDIncrement));
    }

    /*public void IncrementKp(double new_kp)
    {
        aimSubsystem.servo.setKP(Math.max(0, aimSubsystem.servo.getKP() + new_kp));
    }
    public void IncrementKI(double new_kI)
    {
        aimSubsystem.servo.setKI(Math.max(0, aimSubsystem.servo.getKI() + new_kI));
    }
    public void IncrementKD(double new_kD)
    {
        aimSubsystem.servo.setKD(Math.max(0, aimSubsystem.servo.getKD() + new_kD));
    }*/

    @Override
    public boolean isFinished() {
        return true;
    }
}
