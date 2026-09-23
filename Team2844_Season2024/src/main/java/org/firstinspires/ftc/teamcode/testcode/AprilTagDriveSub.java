
package org.firstinspires.ftc.team12841.testcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

//TESTCODE
public class AprilTagDriveSub extends SubsystemBase {
    public DcMotor m_frontLeft;
    public DcMotor m_frontRight;
    public DcMotor m_backLeft;
    public DcMotor m_backRight;
    public final double SPEED_GAIN  =  0.1  ;   //  was 0.25, Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public final double STRAFE_GAIN =  0.0025 ;   // was 0.05, was 0.1, Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public final double TURN_GAIN   =  0.3 ;//was, .1, was 0.025, was 0.05  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public final double DESIRED_DISTANCE = 2.0;//  this is how close the camera should get to the target (inches)
    public final double MAX_AUTO_SPEED = 1;   // 0.5 Clip the approach speed to this max value (adjust for your robot)
    public final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    //public final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    public final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    public AprilTagDriveSub(DcMotor frontLeft, DcMotor frontRight, DcMotor backRight, DcMotor backLeft){
        m_frontLeft = frontLeft;
        m_frontRight = frontRight;
        m_backLeft = backLeft;
        m_backRight = backRight;
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
          m_frontLeft.setPower(leftFrontPower);
          m_frontRight.setPower(rightFrontPower);
          m_backLeft.setPower(leftBackPower);
          m_backRight.setPower(rightBackPower);
    }



    @Override
    public void periodic(){

    }
}