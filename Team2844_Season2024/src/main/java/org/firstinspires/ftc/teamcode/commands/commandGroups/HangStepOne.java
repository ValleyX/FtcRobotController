package org.firstinspires.ftc.team12841.commands.commandGroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.commands.hangcommands.HangOut;
import org.firstinspires.ftc.team12841.commands.hangcommands.HangStart;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftOut;
import org.firstinspires.ftc.team12841.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.team12841.subsystems.LiftSubsystem;

/**first step of the hang sequence*/
public class HangStepOne extends SequentialCommandGroup {
    public boolean hangDone = false;
    RevBlinkinLedDriver m_blinkin;
    LiftSubsystem m_lift;
    IntakeSubsystem m_intake;
    public HangStepOne(LiftSubsystem liftSubsystem, RevBlinkinLedDriver blinkin, LinearOpMode opMode, IntakeSubsystem intake){
        m_blinkin = blinkin;
        m_lift = liftSubsystem;
        m_intake = intake;

        m_intake.setIntakeDropServo(RobotHardware.INTAKE_SERVO_INIT);
        // This will set the lift and hang to the correct positions before driving to hang bar
        addCommands(
               new HangStart(m_lift,opMode),  //release hang hooks and raise low hang lift
                new HangOut(m_lift,opMode),  //move hang slides to start pos
                new LiftOut(m_lift,opMode), //move lift slides to start pos
                new HangStepTwo(m_lift, blinkin, opMode)//plan to implement touch buttons


        );



        addRequirements(liftSubsystem);
    }



    @Override
    public void end(boolean interrupted) {
        m_blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        hangDone = true;
    }
}
