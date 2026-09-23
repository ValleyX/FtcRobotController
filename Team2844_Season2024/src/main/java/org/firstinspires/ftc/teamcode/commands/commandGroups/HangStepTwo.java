package org.firstinspires.ftc.team12841.commands.commandGroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.commands.hangcommands.HangExtend;
import org.firstinspires.ftc.team12841.commands.hangcommands.HangReset;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftIn;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftReset;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftTotalReset;
import org.firstinspires.ftc.team12841.subsystems.LiftSubsystem;

/** second step of hang sequence*/
public class HangStepTwo extends SequentialCommandGroup {
    RevBlinkinLedDriver m_blinkin;
    public HangStepTwo(LiftSubsystem liftSubsystem, RevBlinkinLedDriver blinkin, LinearOpMode opMode){
        m_blinkin = blinkin;
        addCommands(
                new HangExtend(liftSubsystem,opMode), //moves hang out a little mor
                new LiftIn(liftSubsystem,opMode), //pulls robot up using lift
                new LiftReset(liftSubsystem,opMode), //pulls robot all the way up on 1st bar
                new HangReset(liftSubsystem, blinkin, opMode) //pulls robot up on the second bar

                //parallel cmd will run both cmd at same side
                /*new ParallelCommandGroup(
                        new LiftTotalReset(liftSubsystem,opMode),
                        new HangReset(liftSubsystem, opMode)
                )*/
        );

        addRequirements(liftSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        m_blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
    }
}
