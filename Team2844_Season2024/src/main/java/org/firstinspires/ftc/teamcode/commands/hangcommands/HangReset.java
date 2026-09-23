package org.firstinspires.ftc.team12841.commands.hangcommands;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.subsystems.LiftSubsystem;

/** sets hang to original  position*/
public class HangReset extends CommandBase {

    LiftSubsystem m_liftSub;
    LinearOpMode m_opMode;
    RevBlinkinLedDriver m_blinkin;

    public HangReset(LiftSubsystem liftSubsystem, RevBlinkinLedDriver blinkin,LinearOpMode opMode){
        m_liftSub = liftSubsystem;
        m_opMode = opMode;
        m_blinkin = blinkin;
    }


    @Override
    public void initialize(){
        m_liftSub.hangToPosition(0,1);

    }//set hang to base position

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted) {
       // m_blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);


    }
}
