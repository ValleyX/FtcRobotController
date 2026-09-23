package org.firstinspires.ftc.team12841.commands.hangcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.subsystems.LiftSubsystem;

/**This puts the hang out at the start of the hang sequence*/
public class HangStart extends CommandBase {
    LiftSubsystem m_liftSub;
    LinearOpMode m_opMode;
    public HangStart(LiftSubsystem liftSubsystem, LinearOpMode opMode){
        m_liftSub = liftSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){
        //put stuff here to run once

        //Open servo for hang hooks
        m_liftSub.hangServoToPosition(RobotHardware.HANG_SERVO_RELEASED);
        m_opMode.sleep(300);
        //reset servo to latch position
        m_liftSub.hangServoToPosition(RobotHardware.HANG_SERVO_LATCHED);
        m_opMode.sleep(50);
        //move hang lift up to 15 inches full speed
        m_liftSub.hangToPosition(15,1);
        //m_opMode.sleep(3000);

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
