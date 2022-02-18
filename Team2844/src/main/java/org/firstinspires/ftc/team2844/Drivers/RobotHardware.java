package org.firstinspires.ftc.team2844.Drivers;

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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.team2844.TestDrivers.SampleRevBlinkinLedDriver;
import org.firstinspires.ftc.team2844.dogecv.detectors.roverrukus.BluePostAlignDetector;
import org.firstinspires.ftc.team2844.dogecv.detectors.roverrukus.GoldAlignDetector;
import org.firstinspires.ftc.team2844.dogecv.detectors.roverrukus.GoldAlignDetectorTry;

import org.firstinspires.ftc.team2844.dogecv.detectors.roverrukus.RedPostAlignDetector;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;

import java.util.concurrent.TimeUnit;

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
public class RobotHardware
{
//    public static final double P_DRIVE_COEFF = 2;
    public LinearOpMode OpMode_;

    public DcMotor  leftFront;
    public DcMotor  rightFront;
    public DcMotor  leftBack;
    public DcMotor duckySpinner;
    public DcMotor  rightBack;
    public DcMotor liftmotor;
    public DcMotor superintake;
    public DistanceSensor sensorRange;
    public DistanceSensor blocksensor;

    public ColorSensor light;




    public WebcamName WebcamDown; //
    public WebcamName WebcamUp; //
    public OpenCvSwitchableWebcam switchableWebcam;
    public SkystoneDeterminationPipeline pipeline;
    public GoldAlignDetector goldPipeline;
    public RedPostAlignDetector redPipeline;
    public BluePostAlignDetector bluePipeline;

    public DigitalChannel liftdowntouch;
   // public DigitalChannel intaketouch;

    public Servo arm;
    public Servo grab;


    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver blinkinLedDriver2;
    public RevBlinkinLedDriver.BlinkinPattern desiredpattern;
    public RevBlinkinLedDriver.BlinkinPattern desiredpatternlift;
    public RevBlinkinLedDriver.BlinkinPattern actualpattern;
    public RevBlinkinLedDriver.BlinkinPattern actualpatternlift;

    Telemetry.Item patternName;
    Telemetry.Item display;
    SampleRevBlinkinLedDriver.DisplayKind displayKind;
    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;



    protected enum  DisplayKind {
        MANUAL,
        AUTO
    }

    public void init()
    {
        displayKind = SampleRevBlinkinLedDriver.DisplayKind.MANUAL;



       setblinkin(RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE, RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE);

        ledCycleDeadline = new Deadline(LED_PERIOD, TimeUnit.SECONDS);
        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);
    }

    public void setblinkin (RevBlinkinLedDriver.BlinkinPattern desiredpattern,
                            RevBlinkinLedDriver.BlinkinPattern desiredpatternlift) {
        if (desiredpattern != actualpattern) {
            blinkinLedDriver.setPattern(desiredpattern);
            actualpattern = desiredpattern;
        }
        if (desiredpatternlift != actualpatternlift) {
            blinkinLedDriver2.setPattern(desiredpatternlift);
            actualpatternlift = desiredpatternlift;
        }


    }
/*
    public void loop()
    {
        handleGamepad();

        if (displayKind == SampleRevBlinkinLedDriver.DisplayKind.AUTO) {
            doAutoDisplay();
        } else {
            /*
             * MANUAL mode: Nothing to do, setting the pattern as a result of a gamepad event.

        }
    }
    */

    /*
     * handleGamepad
     *
     * Responds to a gamepad button press.  Demonstrates rate limiting for
     * button presses.  If loop() is called every 10ms and and you don't rate
     * limit, then any given button press may register as multiple button presses,
     * which in this application is problematic.
     *
     * A: Manual mode, Right bumper displays the next pattern, left bumper displays the previous pattern.
     * B: Auto mode, pattern cycles, changing every LED_PERIOD seconds.
     */
    /*
    protected void handleGamepad()
    {
        if (!gamepadRateLimit.hasExpired()) {
            return;
        }

        if (OpMode_.gamepad1.a) {
            setDisplayKind(SampleRevBlinkinLedDriver.DisplayKind.MANUAL);
            gamepadRateLimit.reset();
        } else if (OpMode_.gamepad1.b) {
            setDisplayKind(SampleRevBlinkinLedDriver.DisplayKind.AUTO);
            gamepadRateLimit.reset();
        } else if ((displayKind == SampleRevBlinkinLedDriver.DisplayKind.MANUAL) && (OpMode_.gamepad1.left_bumper)) {
            pattern = pattern.previous();
            displayPattern();
            gamepadRateLimit.reset();
        } else if ((displayKind == SampleRevBlinkinLedDriver.DisplayKind.MANUAL) && (OpMode_.gamepad1.right_bumper)) {
            pattern = pattern.next();
            displayPattern();
            gamepadRateLimit.reset();
        }

        else if ((displayKind == SampleRevBlinkinLedDriver.DisplayKind.MANUAL) && (OpMode_.gamepad2.left_bumper)) {
            patternlift = patternlift.previous();
            displayPattern();
            gamepadRateLimit.reset();
        } else if ((displayKind == SampleRevBlinkinLedDriver.DisplayKind.MANUAL) && (OpMode_.gamepad2.right_bumper)) {
            patternlift = patternlift.next();
            displayPattern();
            gamepadRateLimit.reset();
        }
    }

    protected void setDisplayKind(SampleRevBlinkinLedDriver.DisplayKind displayKind)
    {
        this.displayKind = displayKind;
        display.setValue(displayKind.toString());
    }

    protected void doAutoDisplay()
    {
        if (ledCycleDeadline.hasExpired()) {
            pattern = pattern.next();
            displayPattern();
            ledCycleDeadline.reset();
        }
    }

    protected void displayPattern()
    {
        blinkinLedDriver.setPattern(pattern);
        patternName.setValue(pattern.toString());
    }


     */


public enum cameraSelection
    {
        DOWN,
        UP
    }

    public BNO055IMU imu = null;


    //encoder
    public final double     COUNTS_PER_MOTOR_REV    = 28 ;    //  AndyMark Motor Encoder
    public final double     DRIVE_GEAR_REDUCTION    = 20;     // This is < 1.0 if geared UP
    public final double     ONE_MOTOR_COUNT         = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    public final double     Distance_in_one_rev     = 4.0  * Math.PI; //in
    public final double     COUNTS_PER_INCH         = ONE_MOTOR_COUNT / Distance_in_one_rev ;  //TODO determine// in class

    //imu (turny thingy)
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.
    static final double HEADING_THRESHOLD = 2;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.07;     // Larger is more responsive, but also less stable 0.1
    //static final double P_TURN_COEFF = 0.6;     // Larger is more responsive, but also less stable 0.1
    static final double P_DRIVE_COEFF = 0.015;     // Larger is more responsive, but also less stable 0.15

    //lift
    public final double     LIFT_COUNTS_PER_MOTOR_REV    = 28 ;    //  AndyMark Motor Encoder
    public final double     LIFT_DRIVE_GEAR_REDUCTION    = 40.0;     // This is < 1.0 if geared UP  //original was 20
    public final double     LIFT_ONE_MOTOR_COUNT         = LIFT_COUNTS_PER_MOTOR_REV * LIFT_DRIVE_GEAR_REDUCTION;
    public final double     LIFT_DISTANCE_IN_ONE_REV     = 2.7* Math.PI; //actual bot is 9.5
    public final double     LIFT_COUNTS_PER_INCH         = LIFT_ONE_MOTOR_COUNT / LIFT_DISTANCE_IN_ONE_REV ;  //TODO determine// in class

    //lights
    public final static int LED_PERIOD = 10;
    public final static int GAMEPAD_LOCKOUT = 500;





    /* Constructor */
    public RobotHardware(HardwareMap ahwMap, LinearOpMode opMode, int x, int y, cameraSelection camera) {
        /* Public OpMode members. */
        OpMode_ = opMode;

        int cameraMonitorViewId = OpMode_.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", OpMode_.hardwareMap.appContext.getPackageName());
        WebcamDown = OpMode_.hardwareMap.get(WebcamName.class, "Webcam down"); // USB 3.0
        WebcamUp = OpMode_.hardwareMap.get(WebcamName.class, "Webcam up"); // USB 2.0
        pipeline = new RobotHardware.SkystoneDeterminationPipeline(x, y);
        goldPipeline = new GoldAlignDetector();
        redPipeline = new RedPostAlignDetector();
        bluePipeline = new BluePostAlignDetector();

       // switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcamLeft, webcamRight);
        switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId,WebcamDown, WebcamUp);
       // switchableWebcam.openCameraDevice();


        switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                //pick desired camera here
                if (camera == cameraSelection.DOWN) {

                    switchableWebcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
                    switchableWebcam.setActiveCamera(WebcamDown);
                } else {
                    switchableWebcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
                    //switchableWebcam.setActiveCamera(webcamRight);
                    switchableWebcam.setActiveCamera(WebcamUp);
                }


            }




            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        /*
        switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //pick desired camera here
                if (camera == RobotHardware.cameraSelection.LEFT) {
                    switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                    switchableWebcam.setActiveCamera(webcamLeft);
                } else {
                    switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    //switchableWebcam.setActiveCamera(webcamRight);
                    switchableWebcam.setActiveCamera(webcamLeft);
                }
            }
        });
        */

        switchableWebcam.setPipeline(pipeline);
       //switchableWebcam.setPipeline(goldPipeline);
        //switchableWebcam.openCameraDeviceAsync();

        // Define and Initialize Motors
         leftFront = ahwMap.get(DcMotor.class,"leftFront");
         rightFront = ahwMap.get(DcMotor.class,"rightFront");
         leftBack = ahwMap.get(DcMotor.class,"leftBack");
         rightBack = ahwMap.get(DcMotor.class,"rightBack");
        duckySpinner = ahwMap.get(DcMotor.class, "duckSpinner");
         liftmotor = ahwMap.get(DcMotor.class, "liftMotor");
         superintake = ahwMap.get(DcMotor.class, "superintake");

         grab = ahwMap.get(Servo.class,"grab");
         arm = ahwMap.get(Servo.class, "arm");


       liftdowntouch = ahwMap.get(DigitalChannel.class, "liftdowntouch");
        blocksensor = ahwMap.get(DistanceSensor.class, "blocksensor");
        light = ahwMap.get(ColorSensor.class, "light");

        blinkinLedDriver = ahwMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver2 = ahwMap.get(RevBlinkinLedDriver.class, "liftblinkin");

        liftdowntouch.setMode(DigitalChannel.Mode.INPUT);


            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set all motors to zero power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

        // Set all motors to run without encoders by default
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        duckySpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        ///TEST CODE
        while (!OpMode_.isStopRequested() && !imu.isGyroCalibrated()) {
            OpMode_.sleep(50);
            OpMode_.idle();
        }

        if (!imu.isGyroCalibrated()) {
            System.out.println("ValleyX: Gyro not calibrated");
        }

        System.out.println("ValleyX: imu calib status" + imu.getCalibrationStatus().toString());
        OpMode_.telemetry.addData("Mode", "calibrated");
        OpMode_.telemetry.update();
        ///TEST CODE

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


    }


    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        //An enum to define the ring stack size
        public enum MarkerPosition
        {
            Left,
            Middle,
            Right
        }
        //Some color constants
        static final Scalar BLUE = new Scalar(0, 255, 0);
        static final Scalar GREEN = new Scalar(255, 255, 255);
        //Team Color Blue = 109 Average

        //The core values which define the location and size of the sample regions
        //box location and dimensions

        //Point REGION1_TOPLEFT_ANCHOR_POINT;

        Point REGION1_TOPLEFT_ANCHOR_POINTLEFT;
        Point REGION1_TOPLEFT_ANCHOR_POINTMIDDLE;
        Point REGION1_TOPLEFT_ANCHOR_POINTRIGHT;


        static final int REGION_WIDTH = 100;
        static final int REGION_HEIGHT = 100;

        public final int  FOUR_RING_THRESHOLD = 150;

        public final int  ONE_RING_THRESHOLD = 135;

        Point region1Middle_pointA;
        Point region1Middle_pointB;

        Point region1Left_pointA;
        Point region1Left_pointB;

        Point region1Right_pointA;
        Point region1Right_pointB;




        //Working variables
        Mat region1Middle_Cb;
        Mat region1Left_Cb;
        Mat region1Right_Cb;



        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avgMiddle;

        int avgLeft;

        int avgRight;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile SkystoneDeterminationPipeline.MarkerPosition position = SkystoneDeterminationPipeline.MarkerPosition.Middle;
        public volatile int SkystoneAverageMiddle;
        public volatile int SkystoneAverageLeft;
        public volatile int SkystoneAverageRight;


        public SkystoneDeterminationPipeline(int x, int y)
        {
            /*
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(x, y); // 200, 165
            region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
            region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
             */
            REGION1_TOPLEFT_ANCHOR_POINTMIDDLE = new Point(x, y); // 200, 165
            region1Middle_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINTMIDDLE.x, REGION1_TOPLEFT_ANCHOR_POINTMIDDLE.y);
            region1Middle_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINTMIDDLE.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINTMIDDLE.y + REGION_HEIGHT);

            REGION1_TOPLEFT_ANCHOR_POINTLEFT = new Point(2, y); // 200, 165
            region1Left_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINTLEFT.x, REGION1_TOPLEFT_ANCHOR_POINTLEFT.y);
            region1Left_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINTLEFT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINTLEFT.y + REGION_HEIGHT);

            REGION1_TOPLEFT_ANCHOR_POINTRIGHT = new Point(540, y); // 200, 165
            region1Right_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINTRIGHT.x, REGION1_TOPLEFT_ANCHOR_POINTRIGHT.y);
            region1Right_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINTRIGHT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINTRIGHT.y + REGION_HEIGHT);


        }

        //This function takes the RGB frame, converts to YCrCb, and extracts the Cb channel to the 'Cb' variable
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1Middle_Cb = Cb.submat(new Rect(region1Middle_pointA, region1Middle_pointB));
            region1Left_Cb = Cb.submat(new Rect(region1Left_pointA, region1Left_pointB));
            region1Right_Cb = Cb.submat(new Rect(region1Right_pointA, region1Right_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avgMiddle = (int) Core.mean(region1Middle_Cb).val[0];
            avgLeft = (int) Core.mean(region1Left_Cb).val[0];
            avgRight= (int) Core.mean(region1Right_Cb).val[0];



            //middle
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1Middle_pointA, // First point which defines the rectangle
                    region1Middle_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
/*
            SkystoneAverageMiddle = avgMiddle;
            //position = SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
            if(avgMiddle > FOUR_RING_THRESHOLD)
            {
                position = SkystoneDeterminationPipeline.RingPosition.FOUR;
            }
            else if (avgMiddle > ONE_RING_THRESHOLD)
            {
                position = SkystoneDeterminationPipeline.RingPosition.ONE;
            }
            else
            {
                position = SkystoneDeterminationPipeline.RingPosition.NONE;
            }
*/
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1Middle_pointA, // First point which defines the rectangle
                    region1Middle_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill





            //Left
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1Left_pointA, // First point which defines the rectangle
                    region1Left_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                            2); // Thickness of the rectangle lines
/*
            SkystoneAverageLeft = avgLeft;
            //position = SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
                if(avgLeft> FOUR_RING_THRESHOLD)
            {
                position = SkystoneDeterminationPipeline.RingPosition.FOUR;
            }
                else if (avgLeft > ONE_RING_THRESHOLD)
            {
                position = SkystoneDeterminationPipeline.RingPosition.ONE;
            }
                else
            {
                position = SkystoneDeterminationPipeline.RingPosition.NONE;
            }
*/
            Imgproc.rectangle(
                input, // Buffer to draw on
                region1Left_pointA, // First point which defines the rectangl
                region1Left_pointB, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                            -1); // Negative thickness means solid fill






            //Right
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1Right_pointA, // First point which defines the rectangle
                    region1Right_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                            2); // Thickness of the rectangle lines

            SkystoneAverageRight = avgRight;
            //position = SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
            if((avgRight > avgLeft) && (avgRight > avgMiddle))
            {
                position = SkystoneDeterminationPipeline.MarkerPosition.Right;
            }
            else if ((avgLeft > avgRight) && (avgLeft > avgMiddle))
            {
                position = SkystoneDeterminationPipeline.MarkerPosition.Left;
            }
            else
            {
                position = SkystoneDeterminationPipeline.MarkerPosition.Middle;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1Right_pointA, // First point which defines the rectangle
                    region1Right_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                                -1); // Negative thickness means solid fill



                    return input;
        }



        public int getAnalysisMiddle()
        {
            return avgMiddle;
        }

        public int getAnalysisLeft()
        {
            return avgLeft;
        }

        public int getAnalysisRight()
        {
            return avgRight;
        }
    }

    public void zeropower() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public void StraifLeft(double speed) {
        leftFront.setPower(speed);
        leftBack.setPower(-speed);
        rightBack.setPower(speed);
        rightFront.setPower(-speed);
    }

    public void StraifRight(double speed) {
        leftFront.setPower(-speed);
        leftBack.setPower(speed);
        rightBack.setPower(-speed);
        rightFront.setPower( speed);
    }
    public void allpower(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
        rightFront.setPower(power);
    }

    public void SetRedPipeline(){
        switchableWebcam.setPipeline(redPipeline);
        switchableWebcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
    }

    public void SetGoldPipeline(){
        switchableWebcam.setPipeline(goldPipeline);
        switchableWebcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
    }

    public void armdown(){
        arm.setPosition(0.63);
    }

    public void armup() {
       arm.setPosition(0);
    }

    public void grabclose() {
        grab.setPosition(0);
    }

    public void grabopen() {
        grab.setPosition(0.5);
    }

    public void duckySpins(double power) {
        duckySpinner.setPower(power);
    }



}

