package org.firstinspires.ftc.team12841.Drivers;


//Made by Benjamin Ettinger 10/25/23
//driver for LIFT, makes it do the thing :) !!!!


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LiftDrive {

    private RobotHardware robot_; //gets the robot hardware into this class
    //DcMotor liftMotorLeft; //makes motor type
    //DcMotor liftMotorRight; //makes motor type
    ElapsedTime timeRunning; //creates a timer object that we can use anywhere here, like a double but for time

    //constructor
    public LiftDrive(RobotHardware robot) {

        robot_ = robot; //linking (pointing) to instantiated RobotHardwareClass

        //liftMotorLeft = robot_.liftMotorLeft; //gets motor from RobotHardware
        //liftMotorRight = robot_.liftMotorRight; //gets motor from RobotHardware
        timeRunning = new ElapsedTime(); //creates new timer object

    }
    //function to set the lift to a certain height
    //liftInches = how far lift is up in inches
    //liftSpeed = how fast the lift runs 0 to 1
    //errorMargin = how much height error is allowable in the function
    //timeOutMS = how long the program will try to do the thing before it gives up and exits
    //wait = if it is going to stop other functions and just do this, or if it isn't going to do that and basically allows multitasking
    int time;
    public void liftToEncoderCount (int liftCounts, double liftSpeed, int errorMargin, double timeOutMS, boolean wait) {

        //gets the rotations per inch, and then sets the distance that we want the lift to go to
        //int newLeftTarget = robot_.liftMotorLeft.getCurrentPosition() + (int)(liftInches * robot_.LIFT_COUNTS_PER_INCH);
        //int newRightTarget = robot_.liftMotorRight.getCurrentPosition() + (int)(liftInches * robot_.LIFT_COUNTS_PER_INCH);

        time ++;


        //tells motors to use the encoder while running
        robot_.liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sets the target position to the distance we want the lift to go to
        robot_.liftMotorLeft.setTargetPosition(liftCounts);
        robot_.liftMotorRight.setTargetPosition(liftCounts);

        //sets the motors to run to position mode
        robot_.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot_.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //start motion
        robot_.liftMotorLeft.setPower(Math.abs(liftSpeed));
        robot_.liftMotorRight.setPower(Math.abs(liftSpeed));



        //logic to determine if it should wait for the lift to get to the right spot

        //if we want the function to wait, it will stop all other parts of robot until the lift is in the right spot
        if (wait == true) {

            timeRunning.reset();

            while ((robot_.liftMotorLeft.isBusy() && robot_.liftMotorRight.isBusy() /*&& time < 20*/)
                    && (robot_.liftMotorLeft.getCurrentPosition() > (liftCounts + errorMargin)
                    || robot_.liftMotorLeft.getCurrentPosition() < (liftCounts - errorMargin))
                    && (robot_.liftMotorRight.getCurrentPosition() > (liftCounts + errorMargin)
                    || robot_.liftMotorRight.getCurrentPosition() < (liftCounts - errorMargin))
                    || (timeRunning.milliseconds() < timeOutMS))
            {

                robot_.OpMode_.sleep(1); //to yield

            } //while loop end bracket

            //commit STOP
            robot_.liftMotorLeft.setPower(0);
            robot_.liftMotorRight.setPower(0);

        } //if wait==true statement end bracket






    }   //liftToHeight method end bracket



    //function to set the lift to a certain height
        //liftInches = how far lift is up in inches
        //liftSpeed = how fast the lift runs 0 to 1
        //errorMargin = how much height error is allowable in the function
        //timeOutMS = how long the program will try to do the thing before it gives up and exits
        //wait = if it is going to stop other functions and just do this, or if it isn't going to do that and basically allows multitasking
    public void liftToHeight (double liftInches, double liftSpeed, double errorMargin, double timeOutMS, boolean wait) {

        //gets the rotations per inch, and then sets the distance that we want the lift to go to
        int newLeftTarget = robot_.liftMotorLeft.getCurrentPosition() + (int)(liftInches * robot_.LIFT_COUNTS_PER_INCH);
        int newRightTarget = robot_.liftMotorRight.getCurrentPosition() + (int)(liftInches * robot_.LIFT_COUNTS_PER_INCH);

        //tells motors to use the encoder while running
        robot_.liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sets the target position to the distance we want the lift to go to
        robot_.liftMotorLeft.setTargetPosition(newLeftTarget);
        robot_.liftMotorRight.setTargetPosition(newRightTarget);

        //sets the motors to run to position mode
        robot_.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot_.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //start motion
        robot_.liftMotorLeft.setPower(Math.abs(liftSpeed));
        robot_.liftMotorRight.setPower(Math.abs(liftSpeed));



        //logic to determine if it should wait for the lift to get to the right spot

        //if we want the function to wait, it will stop all other parts of robot until the lift is in the right spot
        if (wait == true) {

            timeRunning.reset();

            while ((robot_.liftMotorLeft.isBusy() && robot_.liftMotorRight.isBusy())
                    && (robot_.liftMotorLeft.getCurrentPosition() > (newLeftTarget + errorMargin)
                    || robot_.liftMotorLeft.getCurrentPosition() < (newLeftTarget - errorMargin))
                    && (robot_.liftMotorRight.getCurrentPosition() > (newRightTarget + errorMargin)
                    || robot_.liftMotorRight.getCurrentPosition() < (newRightTarget - errorMargin))
                    || (timeRunning.milliseconds() < timeOutMS))
            {

               robot_.OpMode_.sleep(1); //to yield

            } //while loop end bracket

            //commit STOP
            robot_.liftMotorLeft.setPower(0);
            robot_.liftMotorRight.setPower(0);

        } //if wait==true statement end bracket




    }   //liftToHeight method end bracket



    public boolean isFinished()
    {
        if (robot_.liftMotorLeft.isBusy() || robot_.liftMotorRight.isBusy())
        {
            return false;
        }
        else {
            //commit STOP
            robot_.liftMotorLeft.setPower(0);
            robot_.liftMotorRight.setPower(0);
            return true;
        }
    }


    //steps the lift UP a pixel height (about 3 or so inches I think)
    public void liftStepUp () { //steps the lift up a row

        //check to make sure that we are not at max height
        if (robot_.liftMotorLeft.getCurrentPosition() >= robot_.MAX_LIFT_HEIGHT || robot_.liftMotorRight.getCurrentPosition() >= robot_.MAX_LIFT_HEIGHT) {

            //stop the motors from breaking the robot if we are at the top of our reach
            robot_.liftMotorLeft.setPower(0);
            robot_.liftMotorRight.setPower(0);

        }
        else { //if we aren't going to break ourselves, go do the thing

            //sets lift target
            int newLeftTarget = robot_.liftMotorLeft.getCurrentPosition() + (int)(robot_.LIFT_STEP * robot_.LIFT_COUNTS_PER_INCH);
            int newRightTarget = robot_.liftMotorRight.getCurrentPosition() + (int)(robot_.LIFT_STEP * robot_.LIFT_COUNTS_PER_INCH);
            robot_.OpMode_.telemetry.addData("newLeftTarget", newLeftTarget );
            robot_.OpMode_.telemetry.addData("newRightTarget", newRightTarget );

            //sets the target position to the distance we want the lift to go to
            robot_.liftMotorLeft.setTargetPosition(newLeftTarget);
            robot_.liftMotorRight.setTargetPosition(newRightTarget);

            //tells motor to commit do with go to position mode
            robot_.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot_.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //start motion
            robot_.liftMotorLeft.setPower(Math.abs(robot_.LIFT_SPEED));
            robot_.liftMotorRight.setPower(Math.abs(robot_.LIFT_SPEED));

            //wait until it gets to position
            while (robot_.liftMotorLeft.isBusy() || robot_.liftMotorRight.isBusy()) {
                robot_.OpMode_.sleep(1);
            }

            //resets the motors to run with the encoder
            robot_.liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot_.liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        } //else logic statement end bracket

    }   //liftStepUp method bottom bracket


    //steps the lift DOWN a notch essentially
    public void liftStepDown () { //steps the lift up a row

        if (robot_.liftMotorLeft.getCurrentPosition() <= robot_.MIN_LIFT_HEIGHT || robot_.liftMotorRight.getCurrentPosition() <= robot_.MIN_LIFT_HEIGHT) {

            //stop the motors from breaking the robot if we are at the bottom of our reach
            //0 since that should be the bottom
            robot_.liftMotorLeft.setPower(0);
            robot_.liftMotorRight.setPower(0);

        }
        else { //if we won't break ourselves, then do the thing

            //sets lift target, subtracts the step value since its going down
            int newLeftTarget = robot_.liftMotorLeft.getCurrentPosition() - (int)(robot_.LIFT_STEP * robot_.LIFT_COUNTS_PER_INCH);
            int newRightTarget = robot_.liftMotorRight.getCurrentPosition() - (int)(robot_.LIFT_STEP * robot_.LIFT_COUNTS_PER_INCH);

            //sets the target position to the distance we want the lift to go to
            robot_.liftMotorLeft.setTargetPosition(newLeftTarget);
            robot_.liftMotorRight.setTargetPosition(newRightTarget);

            //tells motor to use run to pos mode when told to run
            robot_.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot_.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //start motion, lift speed is the max speed of the lift for accuracy
            robot_.liftMotorLeft.setPower(Math.abs(robot_.LIFT_SPEED));
            robot_.liftMotorRight.setPower(Math.abs(robot_.LIFT_SPEED));

            //wait until it gets to position
            while (robot_.liftMotorLeft.isBusy() && robot_.liftMotorRight.isBusy()) {
                robot_.OpMode_.sleep(1);
            }

            //resets the motors to run with the encoder
            robot_.liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot_.liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        } //else logic statement end bracket

    }   //liftStepDown method bottom bracket


    //resets lift to min height
    public void liftReset() {

        int newLiftTarget = 0; //sets target to the min height

        //sets the motors to go to the target
        robot_.liftMotorLeft.setTargetPosition(newLiftTarget);
        robot_.liftMotorRight.setTargetPosition(newLiftTarget);

        //makes motor commit do (go to the position)
        robot_.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot_.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //start motion, lift speed is max speed allowed for the lift so it is speedy but accurate
        robot_.liftMotorLeft.setPower(Math.abs(robot_.LIFT_SPEED));
        robot_.liftMotorRight.setPower(Math.abs(robot_.LIFT_SPEED));

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        //wait until it gets to position
        while (robot_.liftMotorLeft.isBusy() && robot_.liftMotorRight.isBusy() && timer.seconds() < 2) {
            robot_.OpMode_.sleep(1);
        }

        //resets the motors to run with the encoder
        robot_.liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //commit STOP
        robot_.liftMotorLeft.setPower(0);
        robot_.liftMotorRight.setPower(0);

    } //liftReset method end bracket


    //moves lift without steps
    public void moveLift(double input) {

        robot_.OpMode_.telemetry.addData("Move Lift Get pos", robot_.liftMotorLeft.getCurrentPosition());
        if (robot_.liftMotorLeft.getCurrentPosition() >= robot_.MAX_LIFT_HEIGHT || robot_.liftMotorLeft.getCurrentPosition() < robot_.MIN_LIFT_HEIGHT){
            robot_.liftMotorLeft.setPower(0);
            robot_.liftMotorRight.setPower(0);
            robot_.OpMode_.telemetry.addData("Move Lift in 0 Get pos", robot_.liftMotorLeft.getCurrentPosition());
        }
        else {


            robot_.liftMotorLeft.setPower(input);
            robot_.liftMotorRight.setPower(input);
        }


    } //moveLift method end bracket

    public void moveLiftNoBounds(double input) {

        robot_.OpMode_.telemetry.addData("Move Lift Get pos", robot_.liftMotorLeft.getCurrentPosition());

        robot_.liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //if (robot_.liftMotorLeft.getCurrentPosition() >= robot_.MAX_LIFT_HEIGHT || robot_.liftMotorLeft.getCurrentPosition() < robot_.MIN_LIFT_HEIGHT){
           // robot_.liftMotorLeft.setPower(0);
            //robot_.liftMotorRight.setPower(0);
            robot_.OpMode_.telemetry.addData("Move Lift in 0 Get pos", robot_.liftMotorLeft.getCurrentPosition());
        //}
        //else {


            robot_.liftMotorLeft.setPower(input);
            robot_.liftMotorRight.setPower(input);
       // }


    } //moveLift method end bracket


}   //class end bracket
