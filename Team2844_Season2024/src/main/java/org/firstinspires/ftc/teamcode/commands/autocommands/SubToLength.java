package org.firstinspires.ftc.team12841.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.subsystems.IntakeSubsystem;

public class SubToLength extends CommandBase {
    IntakeSubsystem m_intake;
    LinearOpMode m_opmode;

    public SubToLength(IntakeSubsystem intake, LinearOpMode opmode) {
        m_intake = intake;
        m_opmode = opmode;

    }


    public void initialize(double inches) {
        //checks to make sure that the subextend doesn't try to overextend
        //if (0 <= inches && inches <= 21) {
            m_intake.subExtendToPosition(inches, 1);
       // }
    }
}
