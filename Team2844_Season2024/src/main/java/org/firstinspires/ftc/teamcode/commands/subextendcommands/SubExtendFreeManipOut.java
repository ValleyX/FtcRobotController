package org.firstinspires.ftc.team12841.commands.subextendcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.subsystems.IntakeSubsystem;

public class SubExtendFreeManipOut extends CommandBase {
    IntakeSubsystem m_intakeSub;
    LinearOpMode m_opMode;
    public SubExtendFreeManipOut(IntakeSubsystem intakeSubsystem, LinearOpMode opMode){
        m_intakeSub = intakeSubsystem;
        m_opMode = opMode;

    }

    @Override
    public void execute(){
        if (m_intakeSub.m_SubExtend.getCurrentPosition() < 41 * RobotHardware.SUBEXTEND_COUNTS_PER_INCH) {//turn on sub extend if not over 41 inches
            m_intakeSub.m_SubExtend.setPower(1);//turn on sub
       }
      else {
            m_intakeSub.m_SubExtend.setPower(0);//disable
        }
    }
    @Override
    public void end(boolean interrupted){
        m_intakeSub.m_SubExtend.setPower(0);//turn off sub

    }
}
