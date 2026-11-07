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


 /*   // This declares the seven motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor intakeMotor;
    DcMotor feederMotor;
    DcMotor drumShooter;
    DcMotor drumShooterTwo; //We are using two motors for the drum shooter so this acts as the right sided one

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    double drumSpeed;
    double intakeSpeed; // this holds a variable speed for tuning the system
    double feederSpeed; // this holds a variable speed for tuning the system*/
    private double drumSpeed = .40;
    private double intakeSpeed = 0.3; // this holds a variable speed for tuning the system
    private double feederSpeed = 1.0; // this holds a variable speed for tuning the system

    @Override
    public void runOpMode() {

        robot.init();

        // Wait for the game to start (driver presses START)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addLine("Press A to reset Yaw");
            telemetry.addLine("Hold left bumper to drive in robot relative");
            telemetry.addLine("The left joystick sets the robot direction");
            telemetry.addLine("Moving the right joystick left and right turns the robot");
            telemetry.addData("DrumSpeed:", "%1.2f", drumSpeed);
            telemetry.addData("drum motor speed", "%f", robot.getDrumSpeed());
            telemetry.addData("intakeSpeed:", "%1.2f", intakeSpeed);
            telemetry.addData("feederSpeed:", "%1.2f", feederSpeed);
            telemetry.update();
            // If you press the A button, then you reset the Yaw to be zero from the way
            // the robot is currently pointing
            if (gamepad1.a) {
                robot.setImuYaw();
            }
            // If you press the left bumper, you get a drive from the point of view of the robot
            // (much like driving an RC vehicle)
            //*************************************
            // Forward would be 2 and 3 which are wrong. Left and right are strafe.
            // rotate would be 1 and 0 which would be wrong
            // 2 and 3 are wrong on one and 0 and 1 are wrong on the other (basically reverse two of them)
            if (gamepad1.right_trigger_pressed) {
                robot.drive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
            } else {
                robot.driveFieldRelative(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
            }


            if (gamepad1.leftBumperWasPressed() ) {
                robot.setFeederMotorPower(feederSpeed);
            }
            if (gamepad1.bWasPressed()) {
                            robot.setFeederMotorPower(0.0); // set feeder power to 0 percent
            }
//            if (gamepad1.left_bumper) {
//                //intakeMotor.setPower(intakeSpeed); // set feeder power to 20 percent
//                robot.setFeederMotorPower(feederSpeed);
//            } else {
//                robot.setFeederMotorPower(0.0); // set feeder power to 0 percent
//            }

            //NOTES  40 to 45% seems to work if drum is at full speed
            if (gamepad1.right_bumper) {
                robot.setDrumSpeedPower(drumSpeed); //drum spins when we hit right bumper

            } //else {
            if (gamepad1.rightBumperWasReleased()) {
                robot.setDrumSpeedPower(0.0); // set shooter power to 0 percent
                robot.setFeederMotorPower(0.0); // set feeder power to 0 percent
               // robot.setDrumSpeedPower(0.0);
            }

            //need more power to feeder to extake, try 100%
            if (gamepad1.left_trigger_pressed) {
                robot.setIntakeMotorPower(-0.75); // extakes the intake in case the objects need to be removed
            } else {
                robot.setIntakeMotorPower(0.0); // stops extaking
            }
            // adjust the speed for drum when up Dpad or Down Dpad was pressed
            if (gamepad1.dpadUpWasPressed()) {
                drumSpeed = Math.min((drumSpeed + 0.01), 1); // Increases drum speed on click
            }
            if (gamepad1.dpadDownWasPressed()) {
                drumSpeed = Math.max((drumSpeed - 0.01), 0); // decreases drum speed on click
            }
            // adjust the intake speed when left Dpad or Right Dpad are pressed
            if (gamepad1.dpadLeftWasPressed()) {
                intakeSpeed = Math.min((intakeSpeed + 0.05), 1);
            }
            if (gamepad1.dpadRightWasPressed()) {
                intakeSpeed = Math.max((intakeSpeed - 0.05), 0);
            }
            // adjust the speed for feeder when X or B is pressed
            if (gamepad1.xWasPressed()) {
                feederSpeed = Math.min((feederSpeed + 0.05), 1);
            }
            if (gamepad1.bWasPressed()) {
                feederSpeed = Math.max((feederSpeed - 0.05), 0);
            }
        }
    }

//    // This routine drives the robot field relative
//    private void driveFieldRelative(double forward, double right, double rotate) {
//        // First, convert direction being asked to drive to polar coordinates
//        double theta = Math.atan2(forward, right);
//        double r = Math.hypot(right, forward);
//
//        // Second, rotate angle by the angle the robot is pointing
//        theta = AngleUnit.normalizeRadians(theta -
//                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
//
//        // Third, convert back to cartesian
//        double newForward = r * Math.sin(theta);
//        double newRight = r * Math.cos(theta);
//
//        // Finally, call the drive method with robot relative forward and right amounts
//        drive(newForward, newRight, rotate);
//    }
//
//    // Thanks to FTC16072 for sharing this code!!
// /*   public void drive(double forward, double right, double rotate) {
//        // This calculates the power needed for each wheel based on the amount of forward,
//        // strafe right, and rotate
//        double frontLeftPower = forward + right + rotate;
//        double frontRightPower = forward - right - rotate;
//        double backRightPower = forward + right - rotate;
//        double backLeftPower = forward - right + rotate;
//
//     */   /*double maxPowerFrontLeft = 1.0;
//        double maxPowerFrontRight = 1.0;
//        double maxPowerBackLeftPower = 1.0;
//        double maxPowerBackRightPower = 1.0;*/
//        double maxPower = 1.0;
//        double maxSpeed = 1.0;  // make this slower for outreaches
//
//        // This is needed to make sure we don't pass > 1.0 to any wheel
//        // It allows us to keep all of the motors in proportion to what they should
//        // be and not get clipped
//        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
//        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
//        maxPower = Math.max(maxPower, Math.abs(backRightPower));
//        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
//       /* maxPowerFrontLeft = Math.max(maxPowerFrontLeft, Math.abs(frontLeftPower));
//        maxPowerFrontRight = Math.max(maxPowerFrontRight, Math.abs(frontRightPower));
//        maxPowerBackLeftPower = Math.max(maxPowerBackLeftPower, Math.abs(backLeftPower));
//        maxPowerBackRightPower = Math.max(maxPowerBackRightPower, Math.abs(backRightPower));
//        */
//
//        // We multiply by maxSpeed so that it can be set lower for outreaches
//        // When a young child is driving the robot, we may not want to allow full
//        // speed.
//        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
//        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
//        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
//        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
//    }
}
