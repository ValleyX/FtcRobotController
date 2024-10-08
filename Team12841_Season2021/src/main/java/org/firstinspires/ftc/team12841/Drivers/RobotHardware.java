package org.firstinspires.ftc.team12841.Drivers;

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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 */
public class RobotHardware {
    LinearOpMode OpMode_;

    DcMotor LfMotor;
    DcMotor RfMotor;
    DcMotor LbMotor;
    DcMotor RbMotor;
    DcMotor LiftMotor;
    public DcMotor WheelMotor;
    public DcMotor InMotor;
    public Servo InServo;
    public SkystoneDeterminationPipeline pipeline;
    public WebcamName webcamLeft; // USB 3.0
    public WebcamName webcamRight; // USB 2.0
    public OpenCvSwitchableWebcam switchableWebcam;

    public enum cameraSelection {
        LEFT,
        RIGHT
    }

    private final double COUNTS_PER_MOTOR_REV = 28;    //  AndyMark Motor Encoder
    private final double DRIVE_GEAR_REDUCTION = 20.0;     // This is < 1.0 if geared UP
    private final double ONE_MOTOR_COUNT_IN_ONE_REV = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    private final double WHEEL_CIRCUMFRANSE = 4.0;
    //private final double PI = 3.14159;
    private final double INCHES_IN_ONE_REV = WHEEL_CIRCUMFRANSE * Math.PI;
    final double COUNTS_PER_INCH = ONE_MOTOR_COUNT_IN_ONE_REV / INCHES_IN_ONE_REV; //TODO determine in class

    private final double LiftCOUNTS_PER_MOTOR_REV = 28;    //  AndyMark Motor Encoder
    private final double LiftDRIVE_GEAR_REDUCTION = 40.0;     // This is < 1.0 if geared UP
    private final double LiftONE_MOTOR_COUNT_IN_ONE_REV = LiftCOUNTS_PER_MOTOR_REV * LiftDRIVE_GEAR_REDUCTION;
    private final double LiftWHEEL_CIRCUMFRANSE = 1.5;
    private final double LiftINCHES_IN_ONE_REV = LiftWHEEL_CIRCUMFRANSE * Math.PI;
    final double LiftCOUNTS_PER_INCH = LiftONE_MOTOR_COUNT_IN_ONE_REV / LiftINCHES_IN_ONE_REV; //TODO determine in class


    //Cam POV
    public static final int CamY = 55;
    public static final int CamX1 = 40;
    public static final int CamX2 = 140;
    public static final int CamX3 = 240;

    /* Constructor */
    public RobotHardware(HardwareMap ahwMap, LinearOpMode opMode, int x, int y, int x2, int y2, int x3, int y3 , final cameraSelection camera) {
        /* Public OpMode members. */
        OpMode_ = opMode;

        // Define and Initialize Motors
        LfMotor = ahwMap.get(DcMotor.class, "LfMotor");
        RfMotor = ahwMap.get(DcMotor.class, "RfMotor");
        LbMotor = ahwMap.get(DcMotor.class, "LbMotor");
        RbMotor = ahwMap.get(DcMotor.class, "RbMotor");
        WheelMotor = ahwMap.get(DcMotor.class, "SpinnerMotor");
        LiftMotor = ahwMap.get(DcMotor.class, "LiftMotor");
        InMotor = ahwMap.get(DcMotor.class, "InMotor");
        InServo = ahwMap.get(Servo.class, "InServo");

        // set the direction of left motor to reverse so wheels go the same directions.
        LfMotor.setDirection(DcMotor.Direction.REVERSE);
        LbMotor.setDirection(DcMotor.Direction.REVERSE);
        RfMotor.setDirection(DcMotor.Direction.FORWARD);
        RbMotor.setDirection(DcMotor.Direction.FORWARD);
        WheelMotor.setDirection(DcMotor.Direction.FORWARD);
        LiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        InMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        LfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        InMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        LfMotor.setPower(0);
        LbMotor.setPower(0);
        RfMotor.setPower(0);
        RbMotor.setPower(0);
        WheelMotor.setPower(0);
        LiftMotor.setPower(0);
        InMotor.setPower(0);
        InServo.setPosition(0.8);


        // Set all motors to run without encoders by default
        LfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        InMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        int cameraMonitorViewId = OpMode_.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", OpMode_.hardwareMap.appContext.getPackageName());

        webcamLeft = OpMode_.hardwareMap.get(WebcamName.class, "Webcam"); // USB 3.0
        // webcamRight = OpMode_.hardwareMap.get(WebcamName.class, "Webcam Right"); // USB 2.0
        pipeline = new SkystoneDeterminationPipeline(x, y, x2, y2, x3, y3);
        //webcam.setPipeline(pipeline);

        switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcamLeft, webcamLeft);
        switchableWebcam.openCameraDevice();


        //OpMode_.sleep(1000);

        switchableWebcam.setPipeline(pipeline);

        //switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        //switchableWebcam.setActiveCamera(webcamFront);
        //final boolean usefront = true;
/* commenting out due to update of easy open in 2021 this was a good call
        switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //pick desired camera here

                if (camera == cameraSelection.LEFT) {
                    switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    switchableWebcam.setActiveCamera(webcamLeft);
                } else {
                    switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                    switchableWebcam.setActiveCamera(webcamRight);
                }
            }
        });

 */
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
        //An enum to define the skystone position
        public enum MarkerPos {
            LEFT,
            CENTER,
            RIGHT
        }

        public MarkerPos markerPos = MarkerPos.LEFT;

        //Some color constants
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        //The core values which define the location and size of the sample regions
        //box location and dimensions
        //static final
        //Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(x,y); // 200, 165
        Point REGION1_TOPLEFT_ANCHOR_POINT;
        Point REGION2_TOPLEFT_ANCHOR_POINT;
        Point REGION3_TOPLEFT_ANCHOR_POINT;

        static final int REGION_WIDTH = 50;
        static final int REGION_HEIGHT = 45;

        public final int FOUR_RING_THRESHOLD = 150;
        public final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA;
        Point region1_pointB;

        Point region2_pointA;
        Point region2_pointB;

        Point region3_pointA;
        Point region3_pointB;


        //Working variables
        Mat region1_Cb;
        Mat region2_Cb;
        Mat region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;
        int avg2;
        int avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile SkystoneDeterminationPipeline.MarkerPos position = MarkerPos.LEFT;

        public SkystoneDeterminationPipeline(int x, int y, int x2, int y2, int x3, int y3) {
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(x, y); // 200, 165
            region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
            region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

            REGION2_TOPLEFT_ANCHOR_POINT = new Point(x2, y2); // 300, 165
            region2_pointA = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x, REGION2_TOPLEFT_ANCHOR_POINT.y);
            region2_pointB = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

            REGION3_TOPLEFT_ANCHOR_POINT = new Point(x3, y3); // 100, 165
            region3_pointA = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x, REGION3_TOPLEFT_ANCHOR_POINT.y);
            region3_pointB = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        }


        //This function takes the RGB frame, converts to YCrCb, and extracts the Cb channel to the 'Cb' variable
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];
            if (avg1 > avg2 && avg1 > avg3){
                markerPos = MarkerPos.LEFT;
            } else if (avg2 > avg1 && avg2 > avg3){
                markerPos = MarkerPos.CENTER;
            } else if (avg3 > avg1 && avg3 > avg2){
                markerPos = MarkerPos.RIGHT;
            }


            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
/*
            //position = SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD)
            {
                position = SkystoneDeterminationPipeline.RingPosition.FOUR;
            }
            else if (avg1 > ONE_RING_THRESHOLD)
            {
                position = SkystoneDeterminationPipeline.RingPosition.ONE;
            }
            else
            {
                position = SkystoneDeterminationPipeline.RingPosition.NONE;
            }
*/
            return input;
        }

        public int getAnalysis() {
            return avg1;
        }



    }

    public void allpower (double power) {
        LbMotor.setPower(power);
        RbMotor.setPower(power);
        LfMotor.setPower(power);
        RfMotor.setPower(power);
    }
}

