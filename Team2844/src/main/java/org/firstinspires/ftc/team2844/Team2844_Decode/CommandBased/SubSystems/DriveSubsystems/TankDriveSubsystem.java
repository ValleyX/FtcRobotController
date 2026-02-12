package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class TankDriveSubsystem extends SubsystemBase {
    //private double leftSpeed, rightSpeed;
    private final DifferentialDrive drive;
    private Motor frontLeft, frontRight, backLeft, backRight;

    MotorGroup leftMotors;
    MotorGroup rightMotors;


    public TankDriveSubsystem(MotorGroup leftMotors, MotorGroup rightMotors){
        this.leftMotors = leftMotors;
        this.rightMotors = rightMotors;

        drive = new DifferentialDrive(leftMotors, rightMotors);
    }

    public void drive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);
    }

    public void arcadeDrive(double forwardSpeed, double turnSpeed){
        drive.arcadeDrive(forwardSpeed, turnSpeed);
    }
}
