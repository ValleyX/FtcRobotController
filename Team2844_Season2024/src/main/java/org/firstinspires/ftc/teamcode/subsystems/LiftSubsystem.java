package org.firstinspires.ftc.team12841.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;

public class LiftSubsystem extends SubsystemBase {
    //declare motors
    public DcMotor m_liftMotor;
    public DcMotor m_hangMotor;
    public Servo m_bucket;
    public Servo m_hangServo;
    public TouchSensor m_liftTouch;
    public TouchSensor m_rightBase;
    public TouchSensor m_leftBase;

    //initialize using constructor
    public LiftSubsystem(DcMotor liftMotor, DcMotor hangMotor, Servo bucket, Servo hangServo,TouchSensor liftTouch, TouchSensor rightBase, TouchSensor leftBase){
        m_liftMotor = liftMotor;
        m_hangMotor = hangMotor;
        m_bucket = bucket;
        m_hangServo = hangServo;
        m_liftTouch = liftTouch;
        m_leftBase = leftBase;
        m_rightBase = rightBase;

    }

    //sets bucket servo to position
    public void bucketToPosition(double position){
        m_bucket.setPosition(position);
    }

    //sets hang servo to position
    public void hangServoToPosition(double position){
        m_hangServo.setPosition(position);
    }

    //sets lift to position
    public void liftToPosition (double liftInches, double liftSpeed) {

        //gets the rotations per inch, and then sets the distance that we want the lift to go to

        int newTarget = (int)(Math.abs(liftInches) * RobotHardware.LIFT_COUNTS_PER_INCH);

        //tells motors to use the encoder while running
        m_liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sets the target position to the distance we want the lift to go to
        m_liftMotor.setTargetPosition(newTarget);

        //sets the motors to run to position mode
        m_liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //start motion
        m_liftMotor.setPower(Math.abs(liftSpeed));
    }

    public void hangToPosition (double hangInches, double hangSpeed) {

        //gets the rotations per inch, and then sets the distance that we want the lift to go to

        int newTarget = (int)(Math.abs(hangInches) * RobotHardware.HANG_COUNTS_PER_INCH);

        //tells motors to use the encoder while running
        m_hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sets the target position to the distance we want the lift to go to
        m_hangMotor.setTargetPosition(newTarget);

        //sets the motors to run to position mode
        m_hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //start motion
        m_hangMotor.setPower(Math.abs(hangSpeed));


    }

    @Override
    public void periodic(){

    }
}