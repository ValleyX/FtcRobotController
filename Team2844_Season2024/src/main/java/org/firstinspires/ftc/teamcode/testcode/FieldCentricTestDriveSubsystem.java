package org.firstinspires.ftc.team12841.testcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

//TESTCODE
public class FieldCentricTestDriveSubsystem extends SubsystemBase {
    public DcMotor m_frontLeft;
    public DcMotor m_frontRight;
    public DcMotor m_backLeft;
    public DcMotor m_backRight;
    public BNO055IMU m_imu;
    public IMU m_newImu;
    public FieldCentricTestDriveSubsystem(DcMotor frontLeft, DcMotor frontRight, DcMotor backRight, DcMotor backLeft,/*BNO055IMU imu*/IMU newImu){

        m_frontLeft = frontLeft;
        m_frontRight = frontRight;
        m_backLeft = backLeft;
        m_backRight = backRight;
        //m_imu = imu;
        m_newImu = newImu;
    }

    public void allPower(double power){
        m_frontLeft.setPower(power);
        m_backRight.setPower(power);
        m_backLeft.setPower(power);
        m_frontRight.setPower(power);
    }

    @Override
    public void periodic(){

    }
}