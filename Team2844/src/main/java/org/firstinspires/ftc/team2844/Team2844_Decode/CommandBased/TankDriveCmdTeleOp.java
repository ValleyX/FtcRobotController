package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCmdArcade;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCmdTank;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCommand;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.TankDriveSubsystem;

@TeleOp(name = "Tank Drive: Command Based")
public class TankDriveCmdTeleOp extends CommandOpMode {

    private Motor frontLeft, frontRight, backLeft, backRight;
    private TankDriveSubsystem tankDriveSubsystem;
    private DriveCmdTank driveCmdTank;
    private DriveCmdArcade driveCmdArcade;
    private GamepadEx m_driveOp;
    private MotorGroup leftMotorGroup, rightMotorGroup;

    @Override
    public void initialize() {

        //Map HW motors to variables
        frontLeft = new Motor(hardwareMap, "frontLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backLeft = new Motor(hardwareMap, "backLeft");
        backRight = new Motor(hardwareMap, "backRight");

        //Create motor group with Front as the leader
        leftMotorGroup = new MotorGroup(this.frontLeft,this.backLeft);
        rightMotorGroup = new MotorGroup(this.frontRight,this.backRight);

        //Create the gamepad controller
        m_driveOp = new GamepadEx(gamepad1);

        //Create a drive subsystem and pass in the motors
        tankDriveSubsystem = new TankDriveSubsystem(leftMotorGroup,rightMotorGroup);

        //Create a new drive command and pass in the drive subsystem and the gamepad control values
        //driveCmdTank = new DriveCmdTank(tankDriveSubsystem, m_driveOp::getLeftY, m_driveOp::getRightY);

        //Arcade Drive controls
        driveCmdArcade = new DriveCmdArcade(tankDriveSubsystem, m_driveOp::getLeftY, m_driveOp::getRightX);


        IntakeSubsystem intake = new IntakeSubsystem(new Motor(hardwareMap, "IntakeMotor"));

        // button bindings for the intake
        m_driveOp.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new InstantCommand(intake::activate, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));
        m_driveOp.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new InstantCommand(intake::reverse, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));

        //this will make the drive command always run
        register(tankDriveSubsystem);  //good practice to register the default subsystem

        //Set Tank Drive as default drive Command
        //tankDriveSubsystem.setDefaultCommand(driveCmdTank); //command that runs automatically whenever a subsystem is not being used by another command

        //Set Arcade Drive as default drive Command
        tankDriveSubsystem.setDefaultCommand(driveCmdArcade); //command that runs automatically whenever a subsystem is not being used by another command

        // update telemetry every loop
        schedule(new RunCommand(telemetry::update));

    }
}
