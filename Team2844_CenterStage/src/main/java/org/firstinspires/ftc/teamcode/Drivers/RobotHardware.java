package org.firstinspires.ftc.team12841.Drivers;

//import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
//import org.firstinspires.ftc.teamcode.testcode.SampleRevBlinkinLedDriver;
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

public class RobotHardware {

    public LinearOpMode OpMode_; // pointer to the run time operation mode


    // Adjust these numbers to suit your robot.
    public final double DESIRED_DISTANCE = 30.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    /*
    public final double SPEED_GAIN  =  0.25  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public final double STRAFE_GAIN =  0.1 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public final double TURN_GAIN   =  0.05  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
*/
    public final double SPEED_GAIN  =  0.1  ;   //  was 0.25, Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public final double STRAFE_GAIN =  0.0025 ;   // was 0.05, was 0.1, Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public final double TURN_GAIN   =  0.3 ;   //was, .1, was 0.025, was 0.05  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)



    public final double MAX_AUTO_SPEED = 1;   // 0.5 Clip the approach speed to this max value (adjust for your robot)
    public final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    //public final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    public final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    public final double OD_COUNTS_PER_MOTOR_REV = 8192;    //  AndyMark Motor Encoder
    public final double OD_DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    public final double OD_ONE_MOTOR_COUNT = OD_COUNTS_PER_MOTOR_REV * OD_DRIVE_GEAR_REDUCTION;
    public final double OD_Distance_in_one_rev = 2.0 * Math.PI; //in
    public final double OD_COUNTS_PER_INCH = OD_ONE_MOTOR_COUNT / OD_Distance_in_one_rev;
    public static boolean findTag = false; //if this finds the tag, then we use it to turn on/off driving with sticks

    public final double LIFT_COUNTS_PER_MOTOR_REV = 28;    //  AndyMark Motor Encoder
    public final double LIFT_DRIVE_GEAR_REDUCTION = 10;     // This is < 1.0 if geared UP
    public final double LIFT_ONE_MOTOR_COUNT = LIFT_COUNTS_PER_MOTOR_REV * LIFT_DRIVE_GEAR_REDUCTION;
    public final double LIFT_Distance_in_one_rev = 1.5 * Math.PI; //in
    public final double LIFT_COUNTS_PER_INCH = LIFT_ONE_MOTOR_COUNT / LIFT_Distance_in_one_rev;

    public static double CLIMBER_COUNTS_PER_INCH = 0; //NEEDS CHANGING NUMBER OF TICKS PER INCH ON CLIMBER MOTOR

    public static double LIFT_STEP = 3; //amount of inches per step on the lift in inches
    public static double LIFT_SPEED = 0.75; //how fast the lift goes when certain commands in LiftDrive are called
    public static double MAX_LIFT_HEIGHT = 2112; //max height of lift in inches
    public static double MIN_LIFT_HEIGHT = -1; //min lift height of lift in inches
    public static double CLIMB_MOTOR_MAX = 6000; //max height for the climbers
    public static double CLIMB_MOTOR_MIN = 0; //min height for the climbers
    public static double MAX_CLIMB_SPEED = 0.75; //max speed for the climber

    //Added to test Gyroscope -------------------
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 20 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference

    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    public double  targetHeading = 0; //

    public double  headingError  = 0; //
    public double  driveSpeed    = 0;
    public double  turnSpeed     = 0;
    public double  leftFrontSpeed     = 0;
    public double leftBackSpeed = 0;

    public double rightFrontSpeed = 0;

    public double rightBackSpeed = 0;

    public int leftFrontTarget = 0;

    public int leftBackTarget = 0;

    public int rightFrontTarget = 0;

    public int rightBackTarget = 0;

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    public static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy. was 0.4
    public static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    public static final double     HEADING_THRESHOLD       = .5 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03; //was 0.03    // Larger is more responsive, but also less stable
    // -----------------

    public static double delayTimer = 2000; //delay timer for detection

    public final double deadband = 0.2; //later move to RobotHardware or place with all the constants


    //Bucket constants
    public static final double BUCKET_OPEN = .90                             ;//constant for bucket open
    public static final double BUCKET_CLOSED = 1;

    public static final double BUCKET_MIDDLE = (BUCKET_CLOSED + BUCKET_OPEN) / 2;


    //make motors

    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;

    public DcMotor motorFrontLeft;
    public DcMotor motorBackLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorBackRight;
    public DcMotor liftMotorLeft; //motor that runs the lift located on the left
    public DcMotor liftMotorRight; //motor that runs the lift located on the right
    public DcMotor intakeMotor; //motor for intake
    public DcMotor climbMotor; //motor for climber


    //odometry encoders


    //auxillary stuff
    public DcMotor verticalLeft;
    public DcMotor verticalRight;
    public DcMotor horizontal;

     //public IMU imu;

     public BNO055IMU imu;

    public WebcamName camCam;
    public CenterStagePipeline pipeline;
    public OpenCvSwitchableWebcam switchableWebcam;



    public Servo bucketServo;


    //servo to let hang assembly go up
    public Servo hangUnleashServo;


    //drone launcher release servo
    public Servo launcherServo;



    //Code for lights ---------------------------------------------------------------------------

    public final static int LED_PERIOD = 10;

    /*
     * Rate limit gamepad button presses to every 500ms.
     */
    public final static int GAMEPAD_LOCKOUT = 500;

    public RevBlinkinLedDriver blinkinLedDriver; //primary blinkin system

    public RevBlinkinLedDriver winkinLedDriver; //secondary blinkin system
    public RevBlinkinLedDriver.BlinkinPattern pattern;



    //specific LED pattern declaration
    public RevBlinkinLedDriver.BlinkinPattern autoPattern;
    public RevBlinkinLedDriver.BlinkinPattern redPattern;
    public RevBlinkinLedDriver.BlinkinPattern bluePattern;
    public RevBlinkinLedDriver.BlinkinPattern endgamePattern;



   public  Telemetry.Item patternName;
   public  Telemetry.Item display;
    public Deadline ledCycleDeadline;
    public Deadline gamepadRateLimit;



    //code for lights ---------------------------------------------------------------------------------

    public TouchSensor touchSensor;

    public IntegratingGyroscope gyro;
    public NavxMicroNavigationSensor navxMicro;
    public Orientation angles;

    public double saveGryoAngle;

    public RobotHardware(LinearOpMode opMode,boolean checkBlueColorAuto) {
        OpMode_ = opMode;
        saveGryoAngle = 0;

        // Declare our motors
        // Make sure your ID's match your configuration
        leftFrontDrive = OpMode_.hardwareMap.dcMotor.get("leftFront");
        leftBackDrive = OpMode_.hardwareMap.dcMotor.get("leftBack");
        rightFrontDrive = OpMode_.hardwareMap.dcMotor.get("rightFront");
        rightBackDrive = OpMode_.hardwareMap.dcMotor.get("rightBack");
/*
        //Declare Odometry encoders
        //make sure they match the names of the motors they are linked to
        verticalLeft = OpMode_.hardwareMap.dcMotor.get("leftFront");
        verticalRight = OpMode_.hardwareMap.dcMotor.get("rightFront");
        horizontal = OpMode_.hardwareMap.dcMotor.get("leftBack");

*/
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
/*
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
*/
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Resets the encoder
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        //declare liftMotor Stuff
        liftMotorLeft = OpMode_.hardwareMap.dcMotor.get("liftMotorLeft");
        liftMotorRight = OpMode_.hardwareMap.dcMotor.get("liftMotorRight");

        liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //declare intakeMotor Stuff
        intakeMotor = OpMode_.hardwareMap.dcMotor.get("intakeMotor");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //declare climber stuff
        climbMotor = OpMode_.hardwareMap.dcMotor.get("climberMotor");

        climbMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        climbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //declare bucket servo
        bucketServo = OpMode_.hardwareMap.servo.get("bucketServo");
        bucketServo.setPosition(BUCKET_CLOSED);

        //declare release servo
        hangUnleashServo = OpMode_.hardwareMap.servo.get("hangUnleashServo");
        hangUnleashServo.setPosition(0);

        //declare drone launcher servo
        launcherServo = OpMode_.hardwareMap.servo.get("launcherServo");
        launcherServo.setPosition(0);


       // hangUnleashServo = OpMode_.hardwareMap.servo.get("");


        //external imu
        navxMicro = OpMode_.hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope)navxMicro;


        //touch sensor for lift
       // touchSensor = OpMode_.hardwareMap.get(TouchSensor.class, "sensor_touch");

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        //RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = OpMode_.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        //camera and LED declaration

        camCam = OpMode_.hardwareMap.get(WebcamName.class, "Webcamcolor");

        blinkinLedDriver = OpMode_.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"); //Setting led map

        winkinLedDriver = OpMode_.hardwareMap.get(RevBlinkinLedDriver.class, "winkin"); //2nd LED map


        //LED patterns

        autoPattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        redPattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;
        bluePattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE;
        endgamePattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD;


       // blinkinLedDriver.setPattern(autoPattern);
        winkinLedDriver.setPattern(autoPattern);




        pipeline = new RobotHardware.CenterStagePipeline( checkBlueColorAuto);
        int cameraMonitorViewId = OpMode_.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", OpMode_.hardwareMap.appContext.getPackageName());


        switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, camCam, camCam);

         angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


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




        // Wait for the game to start (Display Gyro value while waiting)


        // Set the encoders for closed loop speed control, and reset the heading.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    //Camera --------------------------------------------------------------------------------------------

    public static class CenterStagePipeline extends OpenCvPipeline {

        public enum DetectionPosition {
            Left,
            Middle,
            Right
        }

        //Some color constants, sets definitions for colors
        //R represents Red and B represents Blue
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar RED = new Scalar(255, 0, 0);

        Point REGION1_TOPLEFT_ANCHOR_POINT,REGION2_TOPLEFT_ANCHOR_POINT, REGION3_TOPLEFT_ANCHOR_POINT;

        static final int REGION_WIDTH = 130;
        static final int REGION_HEIGHT = 33;

        public final int redThreshold = 135; //amount of red the camera can see in a box to assign whether object

        public final int blueThreshold = 135; //amount of blue the camera can see in a box to assign whether object

        Point region1_pointA, region1_pointB, region2_pointA, region2_pointB, region3_pointA,region3_pointB;


        //makes filters for the colors

        Mat region1_R = new Mat();

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

        //Average Red and blue in a region
        public int avgR, avgB, avg2R, avg2B, avg3R, avg3B;



        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile RobotHardware.CenterStagePipeline.DetectionPosition position = RobotHardware.CenterStagePipeline.DetectionPosition.Left;

        boolean checkBlue; //where or not the pipeline checks if regions have blue or not


        //added anchorX and anchorY for test 2023
        public CenterStagePipeline( boolean isBlue) {

            checkBlue = isBlue;

            if(checkBlue) { //starting position is blue
                //anchors to change boxes cordinates if neccary
                REGION1_TOPLEFT_ANCHOR_POINT = new Point(50, 300);
                REGION2_TOPLEFT_ANCHOR_POINT = new Point(350, 300);
                REGION3_TOPLEFT_ANCHOR_POINT = new Point(500, 200);
            }
            else{ //starting position is red
                REGION1_TOPLEFT_ANCHOR_POINT = new Point(0, 300);
                REGION2_TOPLEFT_ANCHOR_POINT = new Point(350, 300);
                REGION3_TOPLEFT_ANCHOR_POINT = new Point(500, 200);
            }

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
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, B, 2);
            Core.extractChannel(YCrCb, R, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_R = R.submat(new Rect(region1_pointA, region1_pointB));
            region1_B = B.submat(new Rect(region1_pointA, region1_pointB));

            region2_R = R.submat(new Rect(region2_pointA, region2_pointB));
            region2_B = B.submat(new Rect(region2_pointA, region2_pointB));

            region3_R = R.submat(new Rect(region3_pointA, region3_pointB));
            region3_B = B.submat(new Rect(region3_pointA, region3_pointB));

        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avgR = (int) Core.mean(region1_R).val[0];
            avgB = (int) Core.mean(region1_B).val[0];

            avg2R = (int) Core.mean(region2_R).val[0];
            avg2B = (int) Core.mean(region2_B).val[0];

            avg3R = (int) Core.mean(region3_R).val[0];//maybe set these to a constant because camera can't see the right size
            avg3B = (int) Core.mean(region3_B).val[0];


            //Left
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            //Center?? could be right
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            //Right?? could be center
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            //compares color of boxes to find greatest value of red, then blue
            //red
            if (checkBlue == false) {
                if (avgR > avg2R && avgR > avg3R && avgR > redThreshold) {
                    position = RobotHardware.CenterStagePipeline.DetectionPosition.Left;
                }
                else if (avg2R > avgR && avg2R > avg3R && avg2R > redThreshold) {
                    position = RobotHardware.CenterStagePipeline.DetectionPosition.Middle;
                }
                else /*if (avg3R > avgR && avg3R > avg2R)*/ {
                    position = RobotHardware.CenterStagePipeline.DetectionPosition.Right;
                }
            }

            //blue
            else {
                if (avgB > avg2B && avgB > avg3B && avgB > blueThreshold) {//value was 140
                    position = RobotHardware.CenterStagePipeline.DetectionPosition.Left;
                }
                else if (avg2B > avgB && avg2B > avg3B && avg2B > blueThreshold) {
                    position = RobotHardware.CenterStagePipeline.DetectionPosition.Middle;
                }
                else  {
                    position = RobotHardware.CenterStagePipeline.DetectionPosition.Right;
                }
            }


            return input;
        }


        public int getAnalysisLeft() {
            return avgB;
        }

        public int getAnalysisRight() {
            return avgR;
        }
    }



    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
//Only used in reference to AprilTag movement but could be used for other motion, so debatable where it should be put
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }



    //set power to the entire robot
    public void allpower(double power) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
    }


    //sets power to the left side
    public void leftPower(double power) {
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
    }

    //sets power to the right side
    public void rightPower(double power) {
        motorBackRight.setPower(power);
        motorFrontRight.setPower(power);

    }

    public void calibrateNavX(){
        OpMode_.telemetry.log().add("Gyro Calibrating. Do Not Move!");

        // Wait until the gyro calibration is complete
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (navxMicro.isCalibrating())  {
            OpMode_.telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            OpMode_.telemetry.update();
            // Thread.sleep(50);
        }
        OpMode_.telemetry.log().clear(); OpMode_.telemetry.log().add("Gyro Calibrated. Press Start.");
        OpMode_.telemetry.clear(); OpMode_.telemetry.update();

        // Wait for the start button to be pressed
        OpMode_.waitForStart();
        OpMode_.telemetry.log().clear();

        while (OpMode_.opModeIsActive()) {

            // Read dimensionalized data from the gyro. This gyro can report angular velocities
            // about all three axes. Additionally, it internally integrates the Z axis to
            // be able to report an absolute angular Z orientation.
            //AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
             angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

           /* telemetry.addLine()
                .addData("dx", formatRate(rates.xRotationRate))
                .addData("dy", formatRate(rates.yRotationRate))
                .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));*/

            OpMode_.telemetry.addLine()
                    .addData("heading", formatAngle(angles.angleUnit, angles.firstAngle))
                    .addData("roll", formatAngle(angles.angleUnit, angles.secondAngle))
                    .addData("pitch", "%s deg", formatAngle(angles.angleUnit, angles.thirdAngle));
            OpMode_.telemetry.update();

            OpMode_.idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public double getNavXHeading(){
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle - saveGryoAngle;
    }

    public void resetNavXHeading(){
       // angles.firstAngle = 0;
        //angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        saveGryoAngle = imu.getAngularOrientation().firstAngle;

    }
}
