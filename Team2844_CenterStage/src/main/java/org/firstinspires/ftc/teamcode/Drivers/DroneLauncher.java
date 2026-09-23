package org.firstinspires.ftc.team12841.Drivers;

/*
    Made by Benjamin Ettinger 12/12/23
    Driver to run the drone launcher on the robot
*/

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DroneLauncher {

    private RobotHardware robot_; //gets the robot hardware into this class

    Servo launchServo; //declares servo in this class


    //constructor
    public DroneLauncher(RobotHardware robot) {

        robot_ = robot; //linking (pointing) to instantiated RobotHardwareClass

        launchServo = robot_.launcherServo; //gets servo from RobotHardware

    } //constructor end bracket


    //method to turn the servo to release the motor
    public void launch(boolean launch) {

        if (launch == true) {
            launchServo.setPosition(0.5);
        }

    } //launch method end bracket


    public void resetLaunchServo(boolean reset) {

        if (reset == true) {
            launchServo.setPosition(0);
        }

    }


} //DoneLauncher class end bracket
