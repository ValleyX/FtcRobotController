package org.firstinspires.ftc.team12841.commands.subextendcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.subsystems.IntakeSubsystem;

/**Turns on intake*/
public class SubExtendIntake extends CommandBase {
    IntakeSubsystem m_intakeSub;
    LinearOpMode m_opMode;
    public SubExtendIntake(IntakeSubsystem intakeSubsystem, LinearOpMode opMode){
        m_intakeSub = intakeSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){
        //turn on intake
        m_intakeSub.intakeOn();
        m_intakeSub.setIntakeDropServo(RobotHardware.INTAKE_SERVO_DOWN);
    }

    @Override
    public void execute(){

        if(m_intakeSub.m_IntakeDropServo.getPosition() < .05){
            m_intakeSub.intakeOff();
        }

    }

    @Override
    public void end(boolean interrupted){
        if(interrupted) {
            m_intakeSub.intakeOff();
            m_intakeSub.setIntakeDropServo(RobotHardware.INTAKE_SERVO_UP);
        }

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}