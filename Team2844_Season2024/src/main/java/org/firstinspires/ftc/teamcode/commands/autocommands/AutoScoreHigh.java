package org.firstinspires.ftc.team12841.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.subsystems.LiftSubsystem;

public class AutoScoreHigh extends CommandBase {

    LiftSubsystem m_liftSub;
    LinearOpMode m_opMode;

    public AutoScoreHigh(LiftSubsystem liftSub, LinearOpMode opMode) {

        m_liftSub = liftSub;
        m_opMode = opMode;

    }

    @Override
    public void initialize(){
        //m_liftSub.liftToPosition(17,1);
        m_liftSub.bucketToPosition(RobotHardware.CLAW_SERVO_ENGAGED); //set the claw to be closed
        m_opMode.sleep(25); //wait so claw fully closes
        m_liftSub.liftToPosition(13,1); //move the lift
        m_opMode.sleep(500); //wait for the lift to finish moving
        m_liftSub.bucketToPosition(RobotHardware.CLAW_SERVO_DISENGAGED); //disengage the claw
        //m_opMode.sleep(100);
    }

}
