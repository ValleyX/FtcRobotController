package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased;

//import com.arcrobotics.ftclib.command.CommandOpMode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCommand;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;

@TeleOp(name = "Mecanum Drive")
public class MecanumCmdTeleOp extends CommandOpMode {

    private Motor frontLeft, frontRight, backLeft, backRight;
    private DriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;
    private GamepadEx m_driveOp;

    @Override
    public void initialize() {

        //Map HW motors to variables
        frontLeft = new Motor(hardwareMap, "frontLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backLeft = new Motor(hardwareMap, "backLeft");
        backRight = new Motor(hardwareMap, "backRight");

        //Create the gamepad controller
        m_driveOp = new GamepadEx(gamepad1);

        //Create a drive subsystem and pass in the motors
        driveSubsystem = new DriveSubsystem(frontLeft, frontRight, backLeft, backRight);

        //Create a new drive command and pass in the drive subsystem and the gamepad control values
        driveCommand = new DriveCommand(driveSubsystem, m_driveOp::getLeftX, m_driveOp::getLeftY, m_driveOp::getRightX);

        //this will make the drive command always run
        register(driveSubsystem);  //good practice to register the default subsystem
        driveSubsystem.setDefaultCommand(driveCommand); //command that runs automatically whenever a subsystem is not being used by another command



    }
}
