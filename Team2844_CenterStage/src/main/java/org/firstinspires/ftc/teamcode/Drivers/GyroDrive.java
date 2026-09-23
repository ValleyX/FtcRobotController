/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.team12841.Drivers;

import android.graphics.Path;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 *  This OpMode illustrates the concept of driving an autonomous path based on Gyro (IMU) heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code uses the Universal IMU interface so it will work with either the BNO055, or BHI260 IMU.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *  The REV Logo should be facing UP, and the USB port should be facing forward.
 *  If this is not the configuration of your REV Control Hub, then the code should be modified to reflect the correct orientation.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: This code implements the requirement of calling setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  https://ftc-docs.firstinspires.org/field-coordinate-system
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


//Maybe a replacement for Odometry drive, right now not tuned to test robot and robot
//no strafe code added yet
//From Testing, Accurate to within around about 3 degrees for turning
//Have not tested combination of turn and move to a place yet
//Not implemented in any hardware
//Changed so that it works with a 4 motor drive
//Robot changed to accommodate this Gyro drive, ports that 0 -3 connected to the Odemetry where changed to connect to the motor encoders

public class GyroDrive
{
    public RobotHardware robot_;
    public OpMode opMode_;

    public NavxMicroNavigationSensor navxMicro;
    public boolean waitForAllMotors;




    public GyroDrive(RobotHardware robot){
        robot_ = robot;  waitForAllMotors = false;
    }



    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        robot_.targetHeading = desiredHeading;  // Save for telemetry

        //robot_.angles = robot_.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Determine the heading current error
        //robot_.headingError = robot_.targetHeading - robot_.getNavXHeading()/*getHeading()*/;
        robot_.headingError = robot_.targetHeading - robot_.imu.getAngularOrientation().firstAngle;;


        // Normalize the error to be within +/- 180 degrees
        while (robot_.headingError > 180)  robot_.headingError -= 360;
        while (robot_.headingError <= -180) robot_.headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(robot_.headingError * proportionalGain, -1, 1);

    }

    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {
        heading = -heading; //reversig the heading

        // Ensure that the OpMode is still active
        if (robot_.OpMode_.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * robot_.COUNTS_PER_INCH);
            robot_.leftFrontTarget = robot_.leftFrontDrive.getCurrentPosition() + moveCounts;
            robot_.leftBackTarget = robot_.leftBackDrive.getCurrentPosition() + moveCounts;
            robot_.rightFrontTarget = robot_.rightFrontDrive.getCurrentPosition() + moveCounts;
            robot_.rightBackTarget = robot_.rightBackDrive.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            robot_.leftFrontDrive.setTargetPosition(robot_.leftFrontTarget);
            robot_.leftBackDrive.setTargetPosition(robot_.leftBackTarget);
            robot_.rightFrontDrive.setTargetPosition(robot_.rightFrontTarget);
            robot_.rightBackDrive.setTargetPosition(robot_.rightBackTarget);

            robot_.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION); //makes robot motors
            robot_.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot_.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot_.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot_.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot_.leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot_.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot_.rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            //robot_.OpMode_.opModeIsActive() && during 10/24/2024 testing
            if (waitForAllMotors) {
                while (robot_.OpMode_.opModeIsActive() &&
                        //(robot_.leftFrontDrive.isBusy() && robot_.leftBackDrive.isBusy() && robot_.rightFrontDrive.isBusy() && robot_.rightBackDrive.isBusy())) {
                    (robot_.leftFrontDrive.isBusy() || robot_.leftBackDrive.isBusy() || robot_.rightFrontDrive.isBusy() || robot_.rightBackDrive.isBusy())) {

                    // Determine required steering to keep on heading
                    robot_.turnSpeed = getSteeringCorrection(heading, robot_.P_DRIVE_GAIN);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        robot_.turnSpeed *= -1.0;

                    // Apply the turning correction to the current driving speed.
                    moveRobot(robot_.driveSpeed, robot_.turnSpeed);

                    // Display drive status for the driver.
                    sendTelemetry(true);
                }
            } else {

                while (robot_.OpMode_.opModeIsActive() &&
                        (robot_.leftFrontDrive.isBusy() && robot_.leftBackDrive.isBusy() && robot_.rightFrontDrive.isBusy() && robot_.rightBackDrive.isBusy())) {
                    //(robot_.leftFrontDrive.isBusy() || robot_.leftBackDrive.isBusy() || robot_.rightFrontDrive.isBusy() || robot_.rightBackDrive.isBusy())) {

                    // Determine required steering to keep on heading
                    robot_.turnSpeed = getSteeringCorrection(heading, robot_.P_DRIVE_GAIN);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        robot_.turnSpeed *= -1.0;

                    // Apply the turning correction to the current driving speed.
                    moveRobot(robot_.driveSpeed, robot_.turnSpeed);

                    // Display drive status for the driver.
                    sendTelemetry(true);
                }
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);

            robot_.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Makes motors run using encoders
            robot_.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot_.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot_.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        }
    }


    public void moveRobot(double drive, double turn) {
        robot_.driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        robot_.turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        robot_.leftFrontSpeed  = drive - turn;

        robot_.rightFrontSpeed = drive + turn;



        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(robot_.leftFrontSpeed), Math.abs(robot_.rightFrontSpeed));
        if (max > 1.0)
        {
            robot_.leftFrontSpeed /= max;
            robot_.rightFrontSpeed /= max;
        }

        robot_.leftFrontDrive.setPower( robot_.leftFrontSpeed); //setting power to the motors based on speeds
        robot_.leftBackDrive.setPower( robot_.leftFrontSpeed);
        robot_.rightFrontDrive.setPower( robot_.rightFrontSpeed);
        robot_.rightBackDrive.setPower( robot_.rightFrontSpeed);
    }


   /*00 public  double getHeading() {
        YawPitchRollAngles orientation = robot_.imu.getRobotYawPitchRollAngles();

        return orientation.getYaw(AngleUnit.DEGREES);
    }*/

    public void turnToHeading(double maxTurnSpeed, double heading) {

        heading = -heading;  //this is reversed in imu
        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, robot_.P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (robot_.OpMode_.opModeIsActive() && (Math.abs(robot_.headingError) > robot_.HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            robot_.turnSpeed = getSteeringCorrection(heading, robot_.P_TURN_GAIN);

            System.out.println("ValleyX: heading Error " + robot_.headingError);
            sendTelemetry(true);

            // Clip the speed to the maximum permitted value.
            robot_.turnSpeed = Range.clip(robot_.turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, robot_.turnSpeed);

            // Display drive status for the driver.
           // sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     *  Obtain & hold a heading for a finite amount of time
     *  <p>
     *  Move will stop once the requested time has elapsed
     *  <p>
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        heading = -heading;  //this is reversed in imu

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (robot_.OpMode_.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            robot_.turnSpeed = getSteeringCorrection(heading, robot_.P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            robot_.turnSpeed = Range.clip(robot_.turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, robot_.turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    private void sendTelemetry(boolean straight) {

        if (straight) {
            robot_.OpMode_.telemetry.addData("Motion", "Drive Straight");
            robot_.OpMode_.telemetry.addData("Target Pos Lf:Rf:Rb:Lb",  "%7d:%7d:%7d:%7d",
                    robot_.leftFrontTarget, robot_.rightFrontTarget, robot_.rightBackTarget, robot_.leftBackTarget);

            robot_.OpMode_.telemetry.addData("Actual Pos Lf:Rf:Rb:Lb",  "%7d:%7d:%7d:%7d",      robot_.leftFrontDrive.getCurrentPosition(),
                    robot_.rightFrontDrive.getCurrentPosition(),robot_.rightBackDrive.getCurrentPosition(), robot_.leftBackDrive.getCurrentPosition());
        } else {
            robot_.OpMode_.telemetry.addData("Motion", "Turning");
        }

        robot_.OpMode_.telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", robot_.targetHeading, robot_.angles.firstAngle/*getHeading()*/);
        robot_.OpMode_.telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", robot_.headingError, robot_.turnSpeed);
        robot_.OpMode_.telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", robot_.leftFrontSpeed, robot_.rightFrontSpeed);
        robot_.OpMode_.telemetry.update();
    }




    //

}

//preparing notes for control award

//Use camera , use openCv
//use imu in gyroscope drive
//practiced apriltag with camera
//practiced with odemetry with odemtry wheels
//switchable camera and plan to have to 3 cameeras
//
