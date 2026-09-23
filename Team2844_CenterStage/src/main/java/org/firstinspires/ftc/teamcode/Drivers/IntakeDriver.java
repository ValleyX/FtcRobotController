package org.firstinspires.ftc.team12841.Drivers;


//Made by Benjamin Ettinger 10/27/23
//driver for INTAKE, makes it do the thing :) !!!!


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeDriver {

    private RobotHardware robot_; //gets the robot hardware into this class

    DcMotor intakeMotor; //gets motor from RobotHardware


    //constructor
    public IntakeDriver(RobotHardware robot) {

        robot_ = robot; //linking (pointing) to instantiated RobotHardwareClass

        intakeMotor = robot_.intakeMotor; //gets motor from RobotHardware

    }


    public void intakeOn (boolean run, double speed) {   //function to turn the intake on or off, negative should make the intake go in reverse


        //if we want it to be on, then it goes
        if (run == true) {

            //start motion
            intakeMotor.setPower(speed);

        }
        else { //if its not being told to be on, don't be on

            //stops motor
            intakeMotor.setPower(0);

        }


    }   //intakeOn method end bracket



}   //class end bracket
