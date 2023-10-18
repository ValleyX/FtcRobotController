package org.firstinspires.ftc.teamcode.testcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

public class RobotHardwareTestVersion {

    public LinearOpMode OpMode_; // pointer to the run time operation mode


    // Adjust these numbers to suit your robot.
    public final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    public final double SPEED_GAIN  =  0.25  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public final double STRAFE_GAIN =  0.1 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public final double TURN_GAIN   =  0.05  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    public final double MAX_AUTO_SPEED = 1;   // 0.5 Clip the approach speed to this max value (adjust for your robot)
    public final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    public final double OD_COUNTS_PER_MOTOR_REV = 8192;    //  AndyMark Motor Encoder
    public final double OD_DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    public final double OD_ONE_MOTOR_COUNT = OD_COUNTS_PER_MOTOR_REV * OD_DRIVE_GEAR_REDUCTION;
    public final double OD_Distance_in_one_rev = 2.0 * Math.PI; //in
    public final double OD_COUNTS_PER_INCH = OD_ONE_MOTOR_COUNT / OD_Distance_in_one_rev;
    public static boolean findTag = false; //if this finds the tag, then we use it to turn on/off driving with sticks

    public static double delayTimer = 2000; //delay timer for detection

    //make motors
    public DcMotor motorFrontLeft;
    public DcMotor motorBackLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorBackRight;

    //odometry encoders
    public DcMotor verticalLeft;
    public DcMotor verticalRight;
    public DcMotor horizontal;
    public BNO055IMU imu;
    public WebcamName camCam;
    public CenterStagePipeline pipeline;
    public OpenCvSwitchableWebcam switchableWebcam;

    public RobotHardwareTestVersion(LinearOpMode opMode,boolean checkBlueColorAuto) {
        OpMode_ = opMode;

        // Declare our motors
        // Make sure your ID's match your configuration
        motorFrontLeft = OpMode_.hardwareMap.dcMotor.get("leftFront");
        motorBackLeft = OpMode_.hardwareMap.dcMotor.get("leftBack");
        motorFrontRight = OpMode_.hardwareMap.dcMotor.get("rightFront");
        motorBackRight = OpMode_.hardwareMap.dcMotor.get("rightBack");

        //Declare Odometry encoders
        //make sure they match the names of the motors they are linked to
       verticalLeft = OpMode_.hardwareMap.dcMotor.get("leftFront");
        verticalRight = OpMode_.hardwareMap.dcMotor.get("rightFront");
        horizontal = OpMode_.hardwareMap.dcMotor.get("leftBack");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
       // motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);



        // Retrieve the IMU from the hardware map
        imu = OpMode_.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        camCam = OpMode_.hardwareMap.get(WebcamName.class, "Webcamcolor");
        pipeline = new RobotHardwareTestVersion.CenterStagePipeline( checkBlueColorAuto);
        int cameraMonitorViewId = OpMode_.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", OpMode_.hardwareMap.appContext.getPackageName());

        switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, camCam, camCam);


        switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                //pick desired camera here
                if (true) {

                    switchableWebcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    switchableWebcam.setActiveCamera(camCam);
                } else {
                    switchableWebcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    //switchableWebcam.setActiveCamera(webcamRight);
                    switchableWebcam.setActiveCamera(camCam);
                }


            }


            @Override
            public void onError(int errorCode) {
                ///
                //* This will be called if the camera could not be opened
                //
            }
        });

        switchableWebcam.setPipeline(pipeline);





    }

    //Camera --------------------------------------------------------------------------------------------

    public static class CenterStagePipeline extends OpenCvPipeline {

        public enum DetectionPosition {
            Left,
            Middle,
            Right
        }

        //Some color constants, sets deintion for colors
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar RED = new Scalar(255, 0, 0);

        Point REGION1_TOPLEFT_ANCHOR_POINT,REGION2_TOPLEFT_ANCHOR_POINT, REGION3_TOPLEFT_ANCHOR_POINT;

        static final int REGION_WIDTH = 130;
        static final int REGION_HEIGHT = 130;

        Point region1_pointA, region1_pointB, region2_pointA, region2_pointB, region3_pointA,region3_pointB;


        //makes filters for the colors

        Mat region1_Red = new Mat();
        
        Mat region1_B = new Mat();

        //box 2
        Mat region2_R = new Mat();

        Mat region2_B = new Mat();

        //box 3

        Mat region3_R = new Mat();
        Mat region3_B = new Mat();

        Mat R = new Mat();
        Mat B = new Mat();

        Mat YCrCb = new Mat();

        public int avgR, avgLeftB, avgLeft2R, avgLeft2B, avgLeft3R, avgLeft3B; //Todo rename these variables


        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile RobotHardwareTestVersion.CenterStagePipeline.DetectionPosition position = RobotHardwareTestVersion.CenterStagePipeline.DetectionPosition.Left;

        boolean checkBlue;

        //added anchorX and anchorY for test 2023
        public CenterStagePipeline( boolean isBlue) {

            checkBlue = isBlue;

            //anchors to change boxes cordinates if neccary
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(0, 200); // 200, 0
            REGION2_TOPLEFT_ANCHOR_POINT = new Point(250, 200); // 200, 0
            REGION3_TOPLEFT_ANCHOR_POINT = new Point(500, 200); // 200, 0

            //Creating points points for later boxes
            region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
            region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
            region2_pointA = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x, REGION2_TOPLEFT_ANCHOR_POINT.y);
            region2_pointB = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
            region3_pointA = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x, REGION3_TOPLEFT_ANCHOR_POINT.y);
            region3_pointB = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        }

        //This function takes the RGB frame, converts to YCrCb, and extracts the Cb channel to the 'Cb' variable
        void inputToCb(Mat input) {
            //           Core.extractChannel(input, R, 0); //0 = red 1 = green 2 = blue
            //           Core.extractChannel(input, B, 2); //0 = red 1 = green 2 = blue
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, B, 2);
            Core.extractChannel(YCrCb, R, 0);

        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_Red = R.submat(new Rect(region1_pointA, region1_pointB));
            region1_B = B.submat(new Rect(region1_pointA, region1_pointB));

            region2_R = R.submat(new Rect(region2_pointA, region2_pointB));
            region2_B = B.submat(new Rect(region2_pointA, region2_pointB));

            region3_R = R.submat(new Rect(region3_pointA, region3_pointB));
            region3_B = B.submat(new Rect(region3_pointA, region3_pointB));

        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avgR = (int) Core.mean(region1_Red).val[0];
            avgLeftB = (int) Core.mean(region1_B).val[0];

            avgLeft2R = (int) Core.mean(region2_R).val[0];
            avgLeft2B = (int) Core.mean(region2_B).val[0];

            avgLeft3R = (int) Core.mean(region3_R).val[0];
            avgLeft3B = (int) Core.mean(region3_B).val[0];


            //Left
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            //compares color of boxes to find greatest value of red, then blue
            //red
            if (checkBlue == false) {
                if (avgR > avgLeft2R && avgR > avgLeft3R) {
                    position = RobotHardwareTestVersion.CenterStagePipeline.DetectionPosition.Left;
                }
                else if (avgLeft2R > avgR && avgLeft2R > avgLeft3R) {
                    position = RobotHardwareTestVersion.CenterStagePipeline.DetectionPosition.Middle;
                }
                else if (avgLeft3R > avgR && avgLeft3R > avgLeft2R) {
                    position = RobotHardwareTestVersion.CenterStagePipeline.DetectionPosition.Right;
                }
            }

            //blue
            else {
                if (avgLeftB > avgLeft2B && avgLeftB > avgLeft3B) {
                    position = RobotHardwareTestVersion.CenterStagePipeline.DetectionPosition.Left;
                }
                else if (avgLeft2B > avgLeftB && avgLeft2B > avgLeft3B) {
                    position = RobotHardwareTestVersion.CenterStagePipeline.DetectionPosition.Middle;
                }
                else  {
                    position = RobotHardwareTestVersion.CenterStagePipeline.DetectionPosition.Right;
                }
            }


            return input;
        }



        public int getAnalysisLeft() {
            return avgLeftB;
        }

        public int getAnalysisRight() {
            return avgR;
        }
    }


    //set power to the entire robot
    public void allpower(double power) {
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);
        motorFrontRight.setPower(power);
    }

    public void leftPower(double power) {
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
    }

    public void rightPower(double power) {
        motorBackRight.setPower(power);
        motorFrontRight.setPower(power);
    }
}
