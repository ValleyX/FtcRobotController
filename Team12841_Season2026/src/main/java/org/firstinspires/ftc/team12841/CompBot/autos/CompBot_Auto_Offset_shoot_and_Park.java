
package org.firstinspires.ftc.team12841.CompBot.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.CompBot.RobotHardware;

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
 *  In this sample, the heading is reset when the Start button is touched on the Driver Station.
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

@Autonomous(name="Offset Shoot and Park", group="Robot")
//@Disabled
public class CompBot_Auto_Offset_shoot_and_Park extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this, false);

    @Override
    public void runOpMode() {
        robot.init();
        waitForStart();

        /*********************************************************************************
         * put all autonomous code in the following section
         * start at audience and look at hive to shoot pollen
         * after that go and park to gain 5 points
         * already starting with 4 pollen
         *************************************************************************************/

        sleep(2000);
        robot.driveStraight(robot.DRIVE_SPEED, 6.0, 0.0);
        robot.turnToHeading(robot.TURN_SPEED, 25.0);
        robot.setDrumSpeedPower(.43);
        sleep(5000);
        robot.setFeederMotorPower(1);
        sleep(2000);
        robot.driveStraight(robot.DRIVE_SPEED, 2.0, -25.0);
        sleep(2000);
        robot.driveStraight(robot.DRIVE_SPEED, -2.0, -25.0);
        robot.setDrumSpeedPower(0);
        robot.setFeederMotorPower(0);
        robot.turnToHeading(robot.TURN_SPEED, 0.0);
       // robot.driveStraight(robot.DRIVE_SPEED, 107.0, 0.0);
      //  robot.turnToHeading(robot.TURN_SPEED, -90.0);
       // robot.driveStraight(robot.DRIVE_SPEED, -20.0, -90.0);
        //robot.turnToHeading(robot.TURN_SPEED, -90)

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.
    }

}
