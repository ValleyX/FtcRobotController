package org.firstinspires.ftc.team12841.commands.liftcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.subsystems.LiftSubsystem;

/** Resets lift to original position*/
public class LiftReset extends CommandBase {

    LiftSubsystem m_liftSub;
    LinearOpMode m_opMode;
    public LiftReset(LiftSubsystem liftSubsystem, LinearOpMode opMode){
        m_liftSub = liftSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){
        m_liftSub.bucketToPosition(RobotHardware.CLAW_SERVO_DISENGAGED);//open claw to reset
       // m_liftSub.liftToPosition(.5,1);

        m_liftSub.liftToPosition(0,1);//set lift to base posiiton


        //m_opMode.sleep(100);

    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
         return true;
     }
}

