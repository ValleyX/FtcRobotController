package org.firstinspires.ftc.team12841.commands.subextendcommands;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.subsystems.IntakeSubsystem;

/**free movement tof subextend to triggers*/
//todo; don't use
public class SubExtendFreeManip extends CommandBase {
    IntakeSubsystem m_intakeSub;
    LinearOpMode m_opMode;
    public SubExtendFreeManip(IntakeSubsystem intakeSubsystem, LinearOpMode opMode){
        m_intakeSub = intakeSubsystem;
        m_opMode = opMode;
        addRequirements(m_intakeSub);
    }


    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double rt = m_opMode.gamepad1.right_trigger;
        double lt = m_opMode.gamepad1.left_trigger;
        //set power to the triggers
        if(rt > 0){
            m_intakeSub.m_SubExtend.setPower(rt);//sub out
        } else if (lt > 0) {
            m_intakeSub.m_SubExtend.setPower(-lt);//sub in
        } else {
            m_intakeSub.m_SubExtend.setPower(0);//disable if nothing pressed
        }
    }

    @Override
    public void end(boolean interrupted){

    }


}
