package org.firstinspires.ftc.team12841.commands.liftcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.subsystems.LiftSubsystem;

/**This pulls the lift in at the second stage of the hang sequence*/
public class LiftIn extends CommandBase {
    LiftSubsystem m_liftSub;
    LinearOpMode m_opMode;
    public LiftIn(LiftSubsystem liftSubsystem, LinearOpMode opMode){
        m_liftSub = liftSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){
        m_liftSub.liftToPosition(1.5,1);//set lift to a smaller position thain where they were for the hang sequence


    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        //wait for lift to finish
        if(m_liftSub.m_liftMotor.isBusy()){
            return false;
        } else {
            return true;
        }
    }
}
