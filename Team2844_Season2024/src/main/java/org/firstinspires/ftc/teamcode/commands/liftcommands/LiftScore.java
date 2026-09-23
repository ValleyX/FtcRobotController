package org.firstinspires.ftc.team12841.commands.liftcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.subsystems.DriveSubsystem;
import org.firstinspires.ftc.team12841.subsystems.LiftSubsystem;

/**This hooks specimins on chamber bar */
public class LiftScore extends CommandBase {
    LiftSubsystem m_liftSub;
    DriveSubsystem m_driveSub;
    LinearOpMode m_opMode;
    double currentPosInches;
    public LiftScore(LiftSubsystem liftSubsystem, DriveSubsystem driveSub, LinearOpMode opMode){
        m_liftSub = liftSubsystem;
        m_driveSub = driveSub;
        m_opMode = opMode;

        currentPosInches = m_liftSub.m_liftMotor.getCurrentPosition() / RobotHardware.LIFT_COUNTS_PER_INCH;

    }


    @Override
    public void initialize(){
        currentPosInches = m_liftSub.m_liftMotor.getCurrentPosition() / RobotHardware.LIFT_COUNTS_PER_INCH;// get th current position of lift in inches

        //close claw
        m_liftSub.bucketToPosition(RobotHardware.CLAW_SERVO_ENGAGED);
        m_opMode.sleep(50);

        //move lift down a little to score on a chamber
        m_liftSub.liftToPosition(currentPosInches-4,1);
        m_opMode.sleep(500);

        //open claw
        m_liftSub.bucketToPosition(RobotHardware.CLAW_SERVO_DISENGAGED);
        m_opMode.sleep(100);

        //todo; remove if driver's want with subextend this was so we did't get caught with old climb hooks
        m_driveSub.allPower(-.5);
        m_opMode.sleep(200);
        m_driveSub.allPower(0);

        //reset lift
        m_liftSub.liftToPosition(0,1);


    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
