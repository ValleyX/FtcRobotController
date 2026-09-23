package org.firstinspires.ftc.team12841.commands.hangcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.subsystems.LiftSubsystem;

/**This extends the hang out at the second stage of the hang sequence*/
public class HangExtend extends CommandBase {
    LiftSubsystem m_liftSub;
    LinearOpMode m_opMode;
    public HangExtend(LiftSubsystem liftSubsystem, LinearOpMode opMode){
        m_liftSub = liftSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){
        m_liftSub.hangToPosition(28,1);//extends hang to first position
        //m_opMode.sleep(1000);

    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        //waits for hang to finish before moving on
        if(m_liftSub.m_hangMotor.isBusy()){
            return false;
        } else {
            return true;
        }
    }
}

