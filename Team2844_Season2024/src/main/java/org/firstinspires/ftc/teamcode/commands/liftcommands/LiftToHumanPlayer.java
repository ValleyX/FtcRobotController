package org.firstinspires.ftc.team12841.commands.liftcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.subsystems.LiftSubsystem;

/**This lets the lift collect specimins from the human player*/
public class LiftToHumanPlayer extends CommandBase {
    LiftSubsystem m_liftSub;
    LinearOpMode m_opMode;
    public LiftToHumanPlayer(LiftSubsystem liftSubsystem, LinearOpMode opMode){
        m_liftSub = liftSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){

        //move lift up to human player
        m_liftSub.liftToPosition(2,1);
        m_opMode.sleep(100);

        m_liftSub.bucketToPosition(RobotHardware.CLAW_SERVO_ENGAGED);//close claw
        m_opMode.sleep(400);

        //TODO; we can change this if we want, we need to lift up to get sample off the wall
        m_liftSub.liftToPosition(4,1);


    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}