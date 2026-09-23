package org.firstinspires.ftc.team12841.Drivers;


//Made by Benjamin Ettinger 11/3/23
//driver for CLIMBER, makes it do the thing :) !!!!


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



public class ClimberDriver {

    private RobotHardware robot_; //gets the robot hardware into this class

    DcMotor climbMotor; //declares motor in this class
    Servo releaseServo; //declares servo in this class


    //constructor
    public ClimberDriver(RobotHardware robot) {

        robot_ = robot; //linking (pointing) to instantiated RobotHardwareClass

        climbMotor = robot_.climbMotor; //gets motor from RobotHardware

        releaseServo = robot_.hangUnleashServo; //gets servo from RobotHardware

    }


    //DEBUG CODE DEBUG CODE
    public void climberUp(double power) {


        //logic so that we don't break the climbers
        //if the climber is at its MAXIMUM, DONT MOVE IT
//        if (climbMotor.getCurrentPosition() >= robot_.CLIMB_MOTOR_MAX) {
//
//            climbMotor.setPower(0);
//
//        } else {
//
            climbMotor.setPower(power);
//
//        }


    } //climberUp end bracket


    //DEBUG CODE DEBUG CODE
    public void climberDown(double power) {

        //logic so that we don't break the climbers
        //if the climber is at its MINIMUM, DONT MOVE IT
//        if (climbMotor.getCurrentPosition() <= robot_.CLIMB_MOTOR_MIN) {
//
//            climbMotor.setPower(0);
//
//        } else {
//
            climbMotor.setPower(power);
//
//        }

    } //climberDown end bracket

    public void moveClimberNoLimit(double power) {

        //moves climber without limits sprcifically for the reset progRAM

            climbMotor.setPower(power);


    }


    //method to set the climbers to a specific position
    public void setClimberPosition (double inches, double speed) {

        //gets the rotations per inch, and then sets the distance that we want the climber to go to
        int newClimbTarget = climbMotor.getCurrentPosition() + (int)(inches * robot_.CLIMBER_COUNTS_PER_INCH);

        //tells motor to use the encoder while running
        climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sets the target position to the distance we want the climber to go to
        climbMotor.setTargetPosition(newClimbTarget);

        //sets the motor to run to position mode
        climbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //start motion
        climbMotor.setPower(Math.abs(speed));

    } //setClimberPosition end bracket


    //method to reset the climber to minimum position
    public void climberReset() {

        //gets the rotations per inch, and then sets the distance that we want the climber to go to
        int newClimbTarget = (int) robot_.CLIMB_MOTOR_MIN;

        //tells motor to use the encoder while running
        climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sets the target position to the distance we want the climber to go to
        climbMotor.setTargetPosition(newClimbTarget);

        //sets the motor to run to position mode
        climbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //start motion
        climbMotor.setPower(Math.abs(robot_.MAX_CLIMB_SPEED));

    } //climberReset end bracket


    //method to set the climber to its max position
    public void climberMax() {

        //gets the rotations per inch, and then sets the distance that we want the climber to go to
        int newClimbTarget = (int) robot_.CLIMB_MOTOR_MAX;

        //tells motor to use the encoder while running
        climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sets the target position to the distance we want the climber to go to
        climbMotor.setTargetPosition(newClimbTarget);

        //sets the motor to run to position mode
        climbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //start motion
        climbMotor.setPower(Math.abs(robot_.MAX_CLIMB_SPEED));

    } //climberMax end bracket


    //Method to release the climber assembly during end game
    public void releaseClimber() {

        releaseServo.setPosition(1);

    } //releaseClimber End Bracket


} //class ClimberDriver end bracket