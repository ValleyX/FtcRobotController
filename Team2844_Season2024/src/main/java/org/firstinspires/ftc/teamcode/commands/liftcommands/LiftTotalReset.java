package org.firstinspires.ftc.team12841.commands.liftcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.subsystems.LiftSubsystem;

/** Resets lift to original position no matter where it starts and restarts encoder*/

//TODO; do not use for now broken
public class LiftTotalReset extends CommandBase {

    LiftSubsystem m_liftSub;
    LinearOpMode m_opMode;
    public LiftTotalReset(LiftSubsystem liftSubsystem, LinearOpMode opMode){
        m_liftSub = liftSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){




        //m_liftSub.m_liftMotor.setPower(-.5);

        //move the lift down to  touch sensor
        while(!m_liftSub.m_liftTouch.isPressed()){
            m_liftSub.m_liftMotor.setPower(-.5);
        }

        //turn off lift
        m_liftSub.m_liftMotor.setPower(0);

        //reset encoder
        m_liftSub.m_liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_liftSub.m_liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}