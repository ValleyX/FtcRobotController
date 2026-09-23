package org.firstinspires.ftc.team12841.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

//Drive Subsystem
public class DriveSubsystem extends SubsystemBase {
    //declare all motors for drive + IMU
    public DcMotor m_frontLeft;
    public DcMotor m_frontRight;
    public DcMotor m_backLeft;
    public DcMotor m_backRight;
    public IMU m_imu;

    //constructor
    public DriveSubsystem(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, IMU imu){
        m_frontLeft = frontLeft;
        m_backLeft = backLeft;
        m_backRight = backRight;
        m_frontRight = frontRight;
        m_imu = imu;
    }

    public void allPower(double speed){
        //set motor power to speed
        m_frontLeft.setPower(speed);
        m_frontRight.setPower(speed);
        m_backRight.setPower(speed);
        m_backLeft.setPower(speed);
    }

    @Override
    public void periodic(){

    }
}