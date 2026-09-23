package org.firstinspires.ftc.team12841.commands.liftcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.subsystems.LiftSubsystem;

/**This puts lift to high chamber scoring position*/
public class LiftHighChamber extends CommandBase {
    LiftSubsystem m_liftSub;
    LinearOpMode m_opMode;
    public LiftHighChamber(LiftSubsystem liftSubsystem, LinearOpMode opMode){
        m_liftSub = liftSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){
        //sets lift to position for scoring at the high chamber
        m_liftSub.liftToPosition(17,1);
       // m_liftSub.bucketToPosition(RobotHardware.BUCKET_SERVO_ENGAGED);

    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
