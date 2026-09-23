package org.firstinspires.ftc.team12841.commands.subextendcommands;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.subsystems.IntakeSubsystem;

/**This pulls the subextend in including bring the bucket in*/
public class SubExtendIn extends CommandBase {
    IntakeSubsystem m_intakeSub;
    LinearOpMode m_opMode;
    public SubExtendIn(IntakeSubsystem intakeSubsystem, LinearOpMode opMode){
        m_intakeSub = intakeSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){
        //bring up bucket
        m_intakeSub.setIntakeDropServo(RobotHardware.INTAKE_SERVO_INIT);

        //reset sub extend
        m_intakeSub.subExtendToPosition(0,1);

    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
