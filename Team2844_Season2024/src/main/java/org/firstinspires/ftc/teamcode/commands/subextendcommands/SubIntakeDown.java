package org.firstinspires.ftc.team12841.commands.subextendcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.subsystems.IntakeSubsystem;


public class SubIntakeDown extends CommandBase {
    IntakeSubsystem m_intakeSub;
    LinearOpMode m_opMode;
    public SubIntakeDown(IntakeSubsystem intakeSubsystem, LinearOpMode opMode){
        m_intakeSub = intakeSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){

        m_intakeSub.setIntakeDropServo(RobotHardware.INTAKE_SERVO_DOWN);

    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
