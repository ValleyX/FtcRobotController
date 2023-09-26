package org.firstinspires.ftc.teamcode.testcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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

//Code from last year to test (and steal from) stuff for this year before implementing into this years robot hardware
public class RobotHardwareTestVersion {
    public LinearOpMode OpMode_;
    public WebcamName camCam;
    public CenterStagePipeline pipeline;

    public boolean checkBlue;
    //public static final int cameraThreshold = 120;

    public CenterStagePipeline pipelineTwo; //added for test
    public OpenCvSwitchableWebcam switchableWebcam;

    //motors
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

    //test motor
    public DcMotorEx elbowMotor;

    //lift motor
    public DcMotor turnTable;
    public DcMotor winch;
    public DcMotor elbow;
    public DcMotor elbow2;

    // lift servo
    public Servo claw;
    public Servo wrist;


    //odometers
    public DcMotor encoderRight;
    public DcMotor encoderLeft;
    public DcMotor encoderAux;

    //IMU
    public BNO055IMU imu;

    //Claw
    public final double clawOpen = 0.1; //0.56
    public final double clawClose = 0.7; //1


    //driver moters
    public final double COUNTS_PER_MOTOR_REV = 28;    //  AndyMark Motor Encoder
    public final double DRIVE_GEAR_REDUCTION = 20;     // This is < 1.0 if geared UP
    public final double ONE_MOTOR_COUNT = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    public final double Distance_in_one_rev = 4.0 * Math.PI; //in
    public final double COUNTS_PER_INCH = ONE_MOTOR_COUNT / Distance_in_one_rev;  //TODO determine// in class

    //lift extendor
    public final double     LIFT_COUNTS_PER_MOTOR_REV    = 28 ;    //  AndyMark Motor Encoder
    public final double     LIFT_DRIVE_GEAR_REDUCTION    = 10.0;     // This is < 1.0 if geared UP
    public final double     LIFT_ONE_MOTOR_COUNT         = LIFT_COUNTS_PER_MOTOR_REV * LIFT_DRIVE_GEAR_REDUCTION;
    public final double     LIFT_DISTANCE_IN_ONE_REV     = 2.6 * Math.PI; //measure of diameter times PI to find circumference
    public final double     LIFT_COUNTS_PER_INCH         = LIFT_ONE_MOTOR_COUNT / LIFT_DISTANCE_IN_ONE_REV ;  //TODO determine// in class

    //lift elbow
    public final double     LIFT_ELBOW_COUNTS_PER_MOTOR_REV    = 28 ;    //  AndyMark Motor Encoder
    public final double     LIFT_ELBOW_DRIVE_GEAR_REDUCTION    = 270.0;     // This is < 1.0 if geared UP
    public final double     LIFT_ELBOW_ONE_MOTOR_COUNT         = LIFT_ELBOW_COUNTS_PER_MOTOR_REV * LIFT_ELBOW_DRIVE_GEAR_REDUCTION;
    public final double     LIFT_ELBOW_DISTANCE_IN_ONE_REV     = 2.6 * Math.PI; //measure of diameter times PI to find circumference (actual bot is 9.5)
    public final double     LIFT_ELBOW_COUNTS_PER_INCH         = LIFT_ELBOW_ONE_MOTOR_COUNT / LIFT_ELBOW_DISTANCE_IN_ONE_REV ;  //TODO determine// in class
    public final double ticsToPower = 1/(3.5*((LIFT_ELBOW_COUNTS_PER_MOTOR_REV * LIFT_ELBOW_DRIVE_GEAR_REDUCTION) / 4)); //equation  to find how many tics are in 90 deg of the elbow motor
    public final double ticksIn90 = (LIFT_ELBOW_COUNTS_PER_MOTOR_REV * LIFT_ELBOW_DRIVE_GEAR_REDUCTION) / 4;


    /*
    public final double     LIFT_ELBOW_COUNTS_PER_MOTOR_REV    = 8192 ;    //  AndyMark Motor Encoder
    public final double     LIFT_ELBOW_DRIVE_GEAR_REDUCTION    = 1;     // This is < 1.0 if geared UP
 //   public final double     LIFT_ELBOW_ONE_MOTOR_COUNT         = LIFT_ELBOW_COUNTS_PER_MOTOR_REV * LIFT_ELBOW_DRIVE_GEAR_REDUCTION;
 //   public final double     LIFT_ELBOW_DISTANCE_IN_ONE_REV     = 2.6 * Math.PI; //measure of diameter times PI to find circumference (actual bot is 9.5)
 //   public final double     LIFT_ELBOW_COUNTS_PER_INCH         = LIFT_ELBOW_ONE_MOTOR_COUNT / LIFT_ELBOW_DISTANCE_IN_ONE_REV ;  //TODO determine// in class
    public final double ticsToPower = 1/(3*((LIFT_ELBOW_COUNTS_PER_MOTOR_REV * LIFT_ELBOW_DRIVE_GEAR_REDUCTION) / 4)); //equasion  to find how many tics are in 90 deg of the elbow motor
//    public final double ticksIn90 = (LIFT_ELBOW_COUNTS_PER_MOTOR_REV * LIFT_ELBOW_DRIVE_GEAR_REDUCTION) / 4;
    public final double ticksIn90 = (2019) / 4;
*/

    //lift Turntable
    public final double     LIFT_TURNTABLE_COUNTS_PER_MOTOR_REV    = 28 ;    //  AndyMark Motor Encoder
    public final double     LIFT_TURNTABLE_DRIVE_GEAR_REDUCTION    = 50.0;
    public final double     LIFT_TURNTABLE_DRIVE_GEARS_RATIO       = 4.3;
    public final double     LIFT_TURNTABLE_ONE_TURN_REV            = LIFT_TURNTABLE_COUNTS_PER_MOTOR_REV * LIFT_TURNTABLE_DRIVE_GEAR_REDUCTION * LIFT_TURNTABLE_DRIVE_GEARS_RATIO; // one revolution of the turn table
    public final double     LIFT_TURNTABLE_COUNTS_PER_DEGREE       = LIFT_TURNTABLE_ONE_TURN_REV / 360; //gives you the number of counts per 1 degree.


    //public final double     LIFT_TURNTABLE_DISTANCE_IN_ONE_REV     = 2.6 * Math.PI; //measure of diameter times PI to find circumference
    //public final double     LIFT_TURNTABLE_COUNTS_PER_INCH         = LIFT_TURNTABLE_ONE_TURN_REV / LIFT_TURNTABLE_DISTANCE_IN_ONE_REV ;  //TODO determine// in class


    //odometry
    public final double OD_COUNTS_PER_MOTOR_REV = 8192;    //  AndyMark Motor Encoder
    public final double OD_DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    public final double OD_ONE_MOTOR_COUNT = OD_COUNTS_PER_MOTOR_REV * OD_DRIVE_GEAR_REDUCTION;
    public final double OD_Distance_in_one_rev = 2.0 * Math.PI; //in
    public final double OD_COUNTS_PER_INCH = OD_ONE_MOTOR_COUNT / OD_Distance_in_one_rev;

    //Odometry Wheels
    public DcMotor verticalLeft, verticalRight, horizontal;

    RobotHardwareTestVersion(LinearOpMode opMode, boolean isBlue) {
        OpMode_ = opMode;
        checkBlue = isBlue;


        //test cod
        //PID
        //elbowMotor.getVelocity();


        //--------end test coce
        /*//remove motor encoders ... but why?? - archer
        leftFront = OpMode_.hardwareMap.dcMotor.get("leftFront"); //control hub port 0
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBack = OpMode_.hardwareMap.dcMotor.get("leftBack"); //control hub port 1
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront = OpMode_.hardwareMap.dcMotor.get("rightFront"); //control hub port 3
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightBack = OpMode_.hardwareMap.dcMotor.get("rightBack"); //control hub port 2
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/


        //odometry hw map stuff
        String rfName = "rightFront", rbName = "rightBack", lfName = "leftFront", lbName = "leftBack";
        String verticalLeftEncoderName = lfName, verticalRightEncoderName = rfName, horizontalEncoderName = lbName;

        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);


        //lift motor
        turnTable = OpMode_.hardwareMap.dcMotor.get("turnTable");  //expansion hub port 0
        turnTable.setDirection(DcMotor.Direction.REVERSE);
        turnTable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turnTable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        winch = OpMode_.hardwareMap.dcMotor.get("winch"); //expansion hub port
        winch.setDirection(DcMotor.Direction.REVERSE);
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elbow = OpMode_.hardwareMap.dcMotor.get("elbow");//expansion hub port
        elbow.setDirection(DcMotor.Direction.FORWARD);
        //elbow.setDirection(DcMotor.Direction.REVERSE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elbow2 = OpMode_.hardwareMap.dcMotor.get("elbow2");//expansion hub port
        //elbow2.setDirection(DcMotor.Direction.FORWARD);
        elbow2.setDirection(DcMotor.Direction.REVERSE);
        elbow2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        claw = OpMode_.hardwareMap.servo.get("claw"); //control hub servo port 0

        wrist = OpMode_.hardwareMap.servo.get("wrist");//control hub servo port 1

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        // define initialization values for IMU, and then initialize it.
        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
     //   parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;



        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = OpMode_.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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

        camCam = OpMode_.hardwareMap.get(WebcamName.class, "Webcamcolor");
        pipeline = new CenterStagePipeline(50,50, isBlue);
        //pipelineTwo = new RobotHardwareTestVersion.PowerPlayPipeline(10,200,190,190); //added for test of Center stage
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


    public static class CenterStagePipeline extends OpenCvPipeline {


        public enum DetectionPosition {
            Left,
            Middle,
            Right
        }



        //Some color constants, sets deintion for colors
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar RED = new Scalar(255, 0, 0);


        Point REGION1_TOPLEFT_ANCHOR_POINT;
        Point REGION2_TOPLEFT_ANCHOR_POINT;
        Point REGION3_TOPLEFT_ANCHOR_POINT;


        static final int REGION_WIDTH = 130;
        static final int REGION_HEIGHT = 130;


        Point region1Left_pointA;
        Point region1Left_pointB;

        Point region2Left_pointA;

        Point region2Left_pointB;

        Point region3Left_pointA;

        Point region3Left_pointB;



        //Working variables

        //makes filters for the colors
        //box 1
        Mat region1Left_R = new Mat();
        Mat region1Left_B = new Mat();

        //box 2
        Mat region2Left_R = new Mat();
        Mat region2Left_B = new Mat();

        //box 3
        Mat region3Left_R = new Mat();
        Mat region3Left_B = new Mat();

        Mat R = new Mat();
        Mat B = new Mat();

        Mat YCrCb = new Mat();


        int avgLeftR;
        int avgLeftB;

        int avgLeft2R;
        int avgLeft2B;

        int avgLeft3R;
        int avgLeft3B;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile DetectionPosition position = DetectionPosition.Left;

        boolean checkBlue;

        //added anchorX and anchorY for test 2023
        public CenterStagePipeline(int x, int y, boolean isBlue) {

            checkBlue = isBlue;
            /*
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(x, y); // 200, 165
            region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
            region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
             */
            //anchors to change boxes cordinates if neccary
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(0, 200); // 200, 0
            REGION2_TOPLEFT_ANCHOR_POINT = new Point(250, 200); // 200, 0
            REGION3_TOPLEFT_ANCHOR_POINT = new Point(500, 200); // 200, 0


            region1Left_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
            region1Left_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
            region2Left_pointA = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x, REGION2_TOPLEFT_ANCHOR_POINT.y);
            region2Left_pointB = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
            region3Left_pointA = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x, REGION3_TOPLEFT_ANCHOR_POINT.y);
            region3Left_pointB = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

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

           // region1Left_Cb = Cb.submat(new Rect(region1Left_pointA, region1Left_pointB));
           // region1Left_Cr = Cr.submat(new Rect(region1Left_pointA, region1Left_pointB));

            region1Left_R = R.submat(new Rect(region1Left_pointA, region1Left_pointB));
            region1Left_B = B.submat(new Rect(region1Left_pointA, region1Left_pointB));

            region2Left_R = R.submat(new Rect(region2Left_pointA, region2Left_pointB));
            region2Left_B = B.submat(new Rect(region2Left_pointA, region2Left_pointB));

            region3Left_R = R.submat(new Rect(region3Left_pointA, region3Left_pointB));
            region3Left_B = B.submat(new Rect(region3Left_pointA, region3Left_pointB));



        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avgLeftR = (int) Core.mean(region1Left_R).val[0];
            avgLeftB = (int) Core.mean(region1Left_B).val[0];

            avgLeft2R = (int) Core.mean(region2Left_R).val[0];
            avgLeft2B = (int) Core.mean(region2Left_B).val[0];

            avgLeft3R = (int) Core.mean(region3Left_R).val[0];
            avgLeft3B = (int) Core.mean(region3Left_B).val[0];


            //Left
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1Left_pointA, // First point which defines the rectangle
                    region1Left_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2Left_pointA, // First point which defines the rectangle
                    region2Left_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3Left_pointA, // First point which defines the rectangle
                    region3Left_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            //compares color of boxes to find greatest value of red, then blue
            //red
            if (checkBlue == false) {
                if (avgLeftR > avgLeft2R && avgLeftR > avgLeft3R) {
                    position = DetectionPosition.Left;
                }
                else if (avgLeft2R > avgLeftR && avgLeft2R > avgLeft3R) {
                    position = DetectionPosition.Middle;
                }
                else if (avgLeft3R > avgLeftR && avgLeft3R > avgLeft2R) {
                    position = DetectionPosition.Right;
                }
            }

            //blue
            else {
                if (avgLeftB > avgLeft2B && avgLeftB > avgLeft3B) {
                    position = DetectionPosition.Left;
                }
                else if (avgLeft2B > avgLeftB && avgLeft2B > avgLeft3B) {
                    position = DetectionPosition.Middle;
                }
                else /*if (avgLeft3B > avgLeftB && avgLeft3B > avgLeft2B)*/ {
                    position = DetectionPosition.Right;
                }
            }



            return input;
        }



        public int getAnalysisLeft() {
            return avgLeftB;
        }

        public int getAnalysisRight() {
            return avgLeftR;
        }
    }
    public void allpower(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
        rightFront.setPower(power);
    }

    public void leftPower(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
    }

    public void rightPower(double power) {
        rightBack.setPower(power);
        rightFront.setPower(power);
    }


    //odometry
    public void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName) {
        rightFront = OpMode_.hardwareMap.dcMotor.get(rfName);
        rightBack = OpMode_.hardwareMap.dcMotor.get(rbName);
        leftFront = OpMode_.hardwareMap.dcMotor.get(lfName);
        leftBack = OpMode_.hardwareMap.dcMotor.get(lbName);

        verticalLeft = OpMode_.hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = OpMode_.hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = OpMode_.hardwareMap.dcMotor.get(hEncoderName);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //verticalRight.setDirection(DcMotor.Direction.REVERSE);
        //horizontal.setDirection(DcMotor.Direction.REVERSE);

        OpMode_.telemetry.addData("Status", "Hardware Map Init Complete");
        OpMode_.telemetry.update();
    }
/*
    public void pidElbow(int degrees)//test code
    {
        LiftTicksToDegreeesMath liftTicksToDegrees;
        liftTicksToDegrees= new LiftTicksToDegreesMath(this);
        int ticks = (liftTicksToDegrees.liftTicktoDegrees(degrees));

        double Kp = 0.0025;
        double Ki = 0.0002;
        double Kd = 0;

        double setPoint = ticks;

        double integralSum = 0;

        double lastError = 0;
        int count = 0;
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        final double maxSpeed = 0.5;

// Elapsed timer class from SDK, please use it, it's epic
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();
        timer2.reset();
       // while (elbow.getCurrentPosition() >= setPoint+2 || elbow.getCurrentPosition() <= setPoint-2 ) {
       while (timer2.milliseconds() < 5000) {

            // obtain the encoder position
             double encoderPosition = elbow.getCurrentPosition();
            // calculate the error
            double error = setPoint - encoderPosition;

            // rate of change of the error
            double derivative = (error - lastError) / timer.seconds();

            System.out.println("valleyX: set point " + setPoint);
            System.out.println("ValleyX: encoder position " + encoderPosition);
            // sum of all error over time

            integralSum = integralSum + (error * timer.seconds());
            //limits Intergral to not get intergrater wind up
            if(integralSum > maxSpeed)
                integralSum = maxSpeed;
            if(integralSum < -maxSpeed)
                integralSum = -maxSpeed;
            OpMode_.telemetry.addData("IntergralSum",integralSum);
 //           System.out.println("valleyX: IntergralSum"+integralSum);
           double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
           //limits output to the top motor speed which is one
            if(out > maxSpeed)
                out = maxSpeed;
            if(out < -maxSpeed)
                out = -maxSpeed;
            OpMode_.telemetry.addData("out",out);
  //          System.out.println("ValleyX: out"+out);
            OpMode_.telemetry.addData("count",(count++));
            elbow.setPower(out);
            OpMode_.telemetry.addData("target pos", ticks);
            OpMode_.telemetry.addData("currentPos", elbow.getCurrentPosition());
            OpMode_.telemetry.addData("Power", out);

            // reset the timer for next time
            timer.reset();
            OpMode_.telemetry.update();
            OpMode_.sleep(100);
        }
        //makes sure elbow stays were it is at
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setTargetPosition( ticks );
        elbow.setPower(1);

        ElapsedTime timer1 = new ElapsedTime();
        timer1.reset();
        while (timer1.milliseconds() < 1000)
        {
            System.out.println("ValleyX: Current " +  elbow.getCurrentPosition());
            System.out.println("ValleyX: target "+ +  elbow.getTargetPosition());
        }


    }
*/
}
