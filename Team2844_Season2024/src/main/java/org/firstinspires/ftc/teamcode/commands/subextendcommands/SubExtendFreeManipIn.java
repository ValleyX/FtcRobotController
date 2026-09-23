package org.firstinspires.ftc.team12841.commands.subextendcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.subsystems.IntakeSubsystem;

public class SubExtendFreeManipIn extends CommandBase {
    IntakeSubsystem m_intakeSub;
    LinearOpMode m_opMode;
    RobotHardware m_robot;
    public SubExtendFreeManipIn(IntakeSubsystem intakeSubsystem, LinearOpMode opMode, RobotHardware robot_){
        m_intakeSub = intakeSubsystem;
        m_opMode = opMode;
        m_robot = robot_;

    }

    @Override
    public void execute(){
        //software extension limit
        if (!m_robot.subTouch.isPressed()) {//turn on sub extend if not over 41 inches
            m_intakeSub.m_SubExtend.setPower(-1);//turn on sub
        }
        else {
            m_intakeSub.m_SubExtend.setPower(0);//disable
        }
    }
    @Override
    public void end(boolean interrupted){
//        //turn off subextend
        m_intakeSub.m_SubExtend.setPower(0);
//
//        if(interrupted){
//        m_intakeSub.m_SubExtend.setPower(0);
//        }
    }
}
