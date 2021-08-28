package org.firstinspires.ftc.teamcode.Team12841.Drivers;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 *
 *
 */
public class RobotHardware4motors {
    LinearOpMode OpMode_;

    public DcMotor leftDrivefront;
    public DcMotor rightDrivefront;
    public DcMotor leftDriveback;
    public DcMotor rightDriveback;
    public DcMotor shooterfront;
    public DcMotor shooterback;
    public DcMotor mouth;
    public Servo ringpusher;
    public Servo Servoarm;
    public Servo Servohand;
    public Servo bucket;
    public BNO055IMU imu;


    private final double COUNTS_PER_MOTOR_REV = 28;    //  AndyMark Motor Encoder
    private final double DRIVE_GEAR_REDUCTION = 40.0;     // This is < 1.0 if geared UP
    private final double ONE_MOTOR_COUNT = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    private final double Circumference = 3.14 * 4; //4 inches of wheel
    final double COUNTS_PER_INCH = ONE_MOTOR_COUNT / Circumference; //TODO determine in class
    public final double ARMUP_POS = 1;     // Maximum rotational position
    public final double ARMDOWN_POS = 0.1;     // Minimum rotational position
    public final double HANDOPEN_POS = 0.4;     // Maximum rotational position
    public final double HANDCLOSE_POS = 0.2;     // Minimum rotational position
    public final double backringpusher_POS = 0.9;     // Back all the way
    public final double frontringpusher_POS = 0.45;     // Forward all the wa/* Constructor */

    public RobotHardware4motors(HardwareMap ahwMap, LinearOpMode opMode) {
        /* Public OpMode members. */
        OpMode_ = opMode;

        // Define and Initialize Motors
        leftDrivefront = ahwMap.get(DcMotor.class, "lmotorfront");
        rightDrivefront = ahwMap.get(DcMotor.class, "rmotorfront");
        leftDriveback = ahwMap.get(DcMotor.class, "lmotorback");
        rightDriveback = ahwMap.get(DcMotor.class, "rmotorback");
        shooterfront = ahwMap.get(DcMotor.class, "shooterfront");
        shooterback = ahwMap.get(DcMotor.class, "shooterback");
        ringpusher = ahwMap.get(Servo.class, "ringpusher");

        Servoarm = ahwMap.get(Servo.class, "ARM");
        Servohand = ahwMap.get(Servo.class, "HAND");

        leftDrivefront.setDirection(DcMotor.Direction.FORWARD);
        rightDrivefront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveback.setDirection(DcMotor.Direction.FORWARD);
        rightDriveback.setDirection(DcMotor.Direction.REVERSE);

        leftDrivefront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrivefront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDriveback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        power0drive();


        // Set all motors to run without encoders by default
        leftDrivefront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrivefront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    public void power0drive() {
        leftDrivefront.setPower(0);
        leftDriveback.setPower(0);
        rightDrivefront.setPower(0);
        rightDriveback.setPower(0);
    }

    public void ArmsSetWabbleGoal() {
        Servoarm.setPosition(ARMDOWN_POS);
        OpMode_.sleep(1000);
        Servohand.setPosition(HANDOPEN_POS);
        OpMode_.sleep(800);
        Servoarm.setPosition(ARMUP_POS);
        OpMode_.sleep(500);
    }

    public void turnleft(double power) {
        leftDrivefront.setPower(-power);
        leftDriveback.setPower(-power);
        rightDrivefront.setPower(power);
        rightDriveback.setPower(power);
    }

    public void turnright(double power) {
        leftDrivefront.setPower(power);
        leftDriveback.setPower(power);
        rightDrivefront.setPower(-power);
        rightDriveback.setPower(-power);
    }

    public void turntoheading(double power, double wantedheading) {
        double heading;
        double initialheading;
        double turn;
        initialheading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        turn = (wantedheading - initialheading);

        if(turn > 180){
         turn  = turn - 360;
        }
        else if(turn < -180){
            turn = turn + 360;
        }

        if (turn > 0) {

            turnright(power);

            heading = initialheading;

            System.out.println("ValleyX right: " + heading);
            while (heading >= wantedheading) {
                heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                System.out.println("ValleyX right: " + heading);
            }
        }
        if (turn < 0) {

            turnleft(power);

            heading = initialheading;

            System.out.println("ValleyX left: " + heading);
            while (heading <= wantedheading) {
                heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                System.out.println("ValleyX left: " + heading);
            }
        }
        power0drive();
    }
}