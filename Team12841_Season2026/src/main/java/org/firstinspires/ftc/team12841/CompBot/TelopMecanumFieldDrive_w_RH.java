/* Copyright (c) 2025 FIRST. All rights reserved.
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
package org.firstinspires.ftc.team12841.CompBot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@TeleOp(name = "Comp Bot Teleop RH", group = "Robot")
//@Disabled
public class TelopMecanumFieldDrive_w_RH extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);


   //double feederSpeed; // this holds a variable speed for tuning the system*/
    private double drumSpeed = 0.45;
    private double intakeSpeed = 0.3; // this holds a variable speed for tuning the system
    private double feederSpeed = 1.0; // this holds a variable speed for tuning the system

    @Override
    public void runOpMode() {

        //Initializes all the motors in Robot Hardware Class
        robot.init();

        // Wait for the game to start (driver presses START)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addLine("Press A to reset Yaw");
            telemetry.addLine("Hold left bumper to drive in robot relative");
            telemetry.addLine("The left joystick sets the robot direction");
            telemetry.addLine("Moving the right joystick left and right turns the robot");
            telemetry.addData("DrumSpeed:", "%1.2f", robot.getDrumVelocity());
            telemetry.addData("drum motor power", "%1.2f", drumSpeed);
            telemetry.addData("intakeSpeed:", "%1.2f", intakeSpeed);
            telemetry.addData("feederSpeed:", "%1.2f", feederSpeed);
            telemetry.update();

            //  Right Bumper = Power on Drumb to speed
            //  Y Button = Stop drum power
            //  Left Bumper = Turn on Feeder
            //  A Button = Reset IMU orientation
            //  Left Trigger = Turn on Intake
            //  Right Trigger = Reverse Intake (extake)
            //
            //  D-Pad UP =  Increment Drum Speed
            //  D-Pad DOWN = Decrement Drum Speed
            //  D-Pad Left =  Increment Intake Speed
            //  D-Pad DOWN = Decrement Intake Speed
            //  X Button = Increment Feeder Speed
            //  B Button = Decrement Feeder Speed

            // Robot is only Field Centric Drive!!
            robot.driveFieldRelative(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

            // If you press the A button, then you reset the Yaw to be zero from the way
            // the robot is currently pointing
            if (gamepad1.a) {
                robot.setImuYaw();
            }

            // Turn drum on until y is pressed
            if (gamepad1.right_bumper) {
                robot.setDrumSpeedPower(drumSpeed);
                telemetry.addLine("Drum powerup");
            }
            if (gamepad1.yWasPressed()) {
                robot.setDrumSpeedPower(0.0); // set feeder power to 0 percent
                telemetry.addLine("Drum stopped");
            }

            if (gamepad1.left_bumper) {
                //intakeMotor.setPower(intakeSpeed); // set feeder power to 20 percent
                robot.setFeederMotorPower(feederSpeed);
            } else {
                robot.setFeederMotorPower(0.0); // set feeder power to 0 percent
            }

            //NOTES  40 to 45% seems to work if drum is at full speed
//            if (gamepad1.right_bumper) {
//                robot.setDrumSpeedPower(drumSpeed); //drum spins when we hit right bumper
//
//            } //else {
//            if (gamepad1.rightBumperWasReleased()) {
//                robot.setDrumSpeedPower(0.0); // set shooter power to 0 percent
//                robot.setFeederMotorPower(0.0); // set feeder power to 0 percent
//               // robot.setDrumSpeedPower(0.0);
//            }

            //This is for ball intake
            if (gamepad1.left_trigger_pressed) { //This is for intake going forward
                robot.setIntakeMotorPower(0.75);
            }
            else if (gamepad1.right_trigger_pressed) { //This is used in order to extake
                robot.setIntakeMotorPower(-0.75);
            } else {
                robot.setIntakeMotorPower(0.0);
            }
            // adjust the speed for drum when up Dpad or Down Dpad was pressed
            if (gamepad1.dpadUpWasPressed()) {
                drumSpeed = Math.min((drumSpeed + 0.01), 1); // Increases drum speed on click of Top D Pad
            }
            if (gamepad1.dpadDownWasPressed()) {
                drumSpeed = Math.max((drumSpeed - 0.01), 0); // decreases drum speed on click of Bottom D Pad
            }
            // adjust the intake speed when left Dpad or Right Dpad are pressed
            if (gamepad1.dpadLeftWasPressed()) {
                intakeSpeed = Math.min((intakeSpeed + 0.05), 1); //Increases intake speed on click of Left D Pad
            }
            if (gamepad1.dpadRightWasPressed()) {
                intakeSpeed = Math.max((intakeSpeed - 0.05), 0); //Decreases intake speed on click of Right D Pad
            }
            // adjust the speed for feeder when X or B is pressed
            if (gamepad1.xWasPressed()) {
                feederSpeed = Math.min((feederSpeed + 0.05), 1); //Increases feeder speed on click of X
            }
            if (gamepad1.bWasPressed()) {
                feederSpeed = Math.max((feederSpeed - 0.05), 0); //Decreases feeder speed on click of B
            }
        }
    }

}
