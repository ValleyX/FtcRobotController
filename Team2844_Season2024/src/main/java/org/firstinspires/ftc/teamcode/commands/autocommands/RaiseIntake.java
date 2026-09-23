package org.firstinspires.ftc.team12841.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.subsystems.IntakeSubsystem;

public class RaiseIntake extends CommandBase {
    IntakeSubsystem m_intake;
    LinearOpMode m_opmode;

    public RaiseIntake(IntakeSubsystem intake, LinearOpMode opmode) {
        m_intake = intake;
        m_opmode = opmode;

    }


    @Override
    public void initialize() {
        m_intake.intakeOff();
        m_intake.setIntakeDropServo(RobotHardware.INTAKE_SERVO_UP);
    }
}
