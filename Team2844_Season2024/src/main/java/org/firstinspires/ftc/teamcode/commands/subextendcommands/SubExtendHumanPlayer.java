package org.firstinspires.ftc.team12841.commands.subextendcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.subsystems.IntakeSubsystem;

/**sequence to drop specimin off at  human player station*/
public class SubExtendHumanPlayer extends CommandBase {
    IntakeSubsystem m_intakeSub;
    LinearOpMode m_opMode;
    public SubExtendHumanPlayer(IntakeSubsystem intakeSubsystem, LinearOpMode opMode){
        m_intakeSub = intakeSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){
       // m_intakeSub.setIntakeDropServo(RobotHardware.INTAKE_SERVO_SPIT);//move bucket down
        //m_opMode.sleep(200);

        //extake
        m_intakeSub.intakeExtake();
        m_opMode.sleep(500);

        //turn of intake
        m_intakeSub.intakeOff();
        //bring up servo
        m_intakeSub.setIntakeDropServo(RobotHardware.INTAKE_SERVO_UP);

    }

    @Override
    public void execute(){

    }



    @Override
    public boolean isFinished(){
        return true;
    }
}
