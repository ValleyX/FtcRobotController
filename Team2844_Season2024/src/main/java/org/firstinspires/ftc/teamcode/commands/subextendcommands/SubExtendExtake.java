package org.firstinspires.ftc.team12841.commands.subextendcommands;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.subsystems.IntakeSubsystem;

/**Turns on Extake*/
public class SubExtendExtake extends CommandBase {
    IntakeSubsystem m_intakeSub;
    LinearOpMode m_opMode;
    public SubExtendExtake(IntakeSubsystem intakeSubsystem, LinearOpMode opMode){
        m_intakeSub = intakeSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){
        //start extaking
        m_intakeSub.intakeExtake();
    }

    @Override
    public void execute(){
        //TODO make this work
        //if bucket up turn intake off
        if(m_intakeSub.m_IntakeDropServo.getPosition() < .05) {
            m_intakeSub.intakeOff();
        }
    }

    @Override
    public void end(boolean interrupted){
        //turns off intake at the end
        if(interrupted) {
            m_intakeSub.intakeOff();
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
