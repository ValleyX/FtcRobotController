package org.firstinspires.ftc.team12841.commands.hangcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.subsystems.LiftSubsystem;

/**This puts the hang out at the start of the hang sequence*/
public class HangOut extends CommandBase {
    LiftSubsystem m_liftSub;
    LinearOpMode m_opMode;
    public HangOut(LiftSubsystem liftSubsystem, LinearOpMode opMode){
        m_liftSub = liftSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){

        //move hang lift to position
        m_liftSub.hangToPosition(23.5,1);//extends hang a little bit more than initial position
        //m_opMode.sleep(3000);

    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
