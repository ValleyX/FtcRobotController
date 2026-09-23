package org.firstinspires.ftc.team12841.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;

//import org.firstinspires.ftc.teamcode.Drivers.RobotHardware;


public class IntakeSubsystem extends SubsystemBase {
    //declare variables
    public DcMotor m_SubExtend;
    public Servo m_IntakeMotor;
    public Servo m_IntakeDropServo;
    //cameraStuff
   // public WebcamName m_camCam;
    //public CameraPipeline pipeline;
    //public OpenCvSwitchableWebcam switchableWebcam;
    public LinearOpMode m_opMode;
    public NormalizedColorSensor m_ColorSensorBucket;
    public NormalizedColorSensor m_ColorSensorBelow;

    //initialize using constructor
    public IntakeSubsystem(DcMotor subExtend, Servo intakeMotor, Servo intakeDrop /*WebcamName camCam,LinearOpMode opMode/*NormalizedColorSensor colorSensorBucket, NormalizedColorSensor colorSensorBelow*/){
        m_SubExtend = subExtend;
        m_IntakeDropServo = intakeDrop;
        m_IntakeMotor = intakeMotor;

       // m_opMode = opMode;

        //m_ColorSensorBucket = colorSensorBucket;
        //m_ColorSensorBelow = colorSensorBelow;


        //camera stuff
        //m_camCam = camCam;//camCam = OpMode_.hardwareMap.get(WebcamName.class, "Webcamcolor");
        /*m_camCam = m_opMode.hardwareMap.get(WebcamName.class, "Webcamcolor");

        int cameraMonitorViewId = m_opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", m_opMode.hardwareMap.appContext.getPackageName());

        switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, camCam, camCam);

        switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                //pick desired camera here
                if (true) {

                    switchableWebcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    switchableWebcam.setActiveCamera(m_camCam);
                } else {
                    switchableWebcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    //switchableWebcam.setActiveCamera(webcamRight);
                    switchableWebcam.setActiveCamera(m_camCam);
                }


            }


            @Override
            public void onError(int errorCode) {
                ///
                //* This will be called if the camera could not be opened
                //
            }
        });
        switchableWebcam.setPipeline(pipeline);*/
    }

    //sets servo to position
    public void setIntakeDropServo(double position){
            m_IntakeDropServo.setPosition(position);
    }

    //turns on intake
    public void intakeOn(){
        m_IntakeMotor.setPosition(0);
    }

    //Extakes
    public void intakeExtake(){
        m_IntakeMotor.setPosition(1);
    }

    //turns off intake
    public void intakeOff(){
        m_IntakeMotor.setPosition(.5);
    }

    //set subextend to position
    public void subExtendToPosition (double subExtendInches, double subExtendSpeed) {

        //gets the rotations per inch, and then sets the distance that we want the lift to go to
        int newTarget = (int)(/*Math.abs*/(subExtendInches) * RobotHardware.SUBEXTEND_COUNTS_PER_INCH);

        //tells motors to use the encoder while running
        m_SubExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sets the target position to the distance we want the lift to go to
        m_SubExtend.setTargetPosition(newTarget);

        //sets the motors to run to position mode
        m_SubExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //start motion
        m_SubExtend.setPower(Math.abs(subExtendSpeed));

    }


    /*public static class CameraPipeline extends OpenCvPipeline {

          public enum DetectionColor {
               Blue,
                Yellow,
               Red,
                 NONE
         }

             //Some color constants, sets definitions for colors
             //R represents Red and B represents Blue
              static final Scalar BLUEY = new Scalar(0, 0, 255);
              //static final Scalar RED = new Scalar(255, 0, 0);

            Point REGION1_TOPLEFT_ANCHOR_POINT,REGION2_TOPLEFT_ANCHOR_POINT, REGION3_TOPLEFT_ANCHOR_POINT;

             static final int REGION_WIDTH = 200;
                static final int REGION_HEIGHT = 150;

             public final int redThreshold = 135; //amount of red the camera can see in a box to assign whether object

            public final int blueThreshold = 133; //amount of blue the camera can see in a box to assign whether object

            public final int yellowMinBlue = 100;

            Point region1_pointA, region1_pointB, region2_pointA, region2_pointB, region3_pointA,region3_pointB;


            //makes filters for the colors

            Mat region1_R = new Mat();

            Mat region1_B = new Mat();

            //Mat region1_G = new Mat();

            //box 2
        /   /Mat region2_R = new Mat();

            /// Mat region2_B = new Mat();

            //box 3

            //  Mat region3_R = new Mat();
            // Mat region3_B = new Mat();

             Mat R = new Mat();
              Mat B = new Mat();
            // Mat G = new Mat();

              Mat YCrCb = new Mat();

            //Average Red and blue in a region
            public int avgR, avgB, avgG, avg2R, avg2B, avg3R, avg3B;



            // Volatile since accessed by OpMode thread w/o synchronization
            public volatile CameraSubsystemTest.CameraPipeline.DetectionColor color = DetectionColor.Blue;

            boolean checkBlue; //where or not the pipeline checks if regions have blue or not


             //added anchorX and anchorY for test 2023
            public CameraPipeline( boolean isBlue) {

            checkBlue = isBlue;

            if(checkBlue) { //starting position is blue
                //anchors to change boxes cordinates if neccary
                REGION1_TOPLEFT_ANCHOR_POINT = new Point(250, 200);
                /*REGION2_TOPLEFT_ANCHOR_POINT = new Point(350, 300);
                REGION3_TOPLEFT_ANCHOR_POINT = new Point(500, 200);
}
           /* else{ //starting position is red
                REGION1_TOPLEFT_ANCHOR_POINT = new Point(0, 300);
                REGION2_TOPLEFT_ANCHOR_POINT = new Point(350, 300);
                REGION3_TOPLEFT_ANCHOR_POINT = new Point(500, 200);
            }

        //Creating points points for later boxes
        region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
        region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        //region2_pointA = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x, REGION2_TOPLEFT_ANCHOR_POINT.y);
        //region2_pointB = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        //region3_pointA = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x, REGION3_TOPLEFT_ANCHOR_POINT.y);
    //region3_pointB = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

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

                    /*region2_R = R.submat(new Rect(region2_pointA, region2_pointB));
                    region2_B = B.submat(new Rect(region2_pointA, region2_pointB));

            region3_R = R.submat(new Rect(region3_pointA, region3_pointB));
            region3_B = B.submat(new Rect(region3_pointA, region3_pointB));
            }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avgR = (int) Core.mean(region1_R).val[0];
            avgB = (int) Core.mean(region1_B).val[0];

            /*avg2R = (int) Core.mean(region2_R).val[0];
            avg2B = (int) Core.mean(region2_B).val[0];

            avg3R = (int) Core.mean(region3_R).val[0];//maybe set these to a constant because camera can't see the right size
            avg3B = (int) Core.mean(region3_B).val[0];


         //Left
         Imgproc.rectangle(
            input, // Buffer to draw on
            region1_pointA, // First point which defines the rectangle
            region1_pointB, // Second point which defines the rectangle
            BLUEY, // The color the rectangle is drawn in
            2); // Thickness of the rectangle lines

            //Center?? could be right
            /*Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUEY, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            //Right?? could be center
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUEY, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            //compares color of boxes to find greatest value of red, then blue*/
    //red
           /* if (checkBlue == false) {
                if (avgR > avg2R && avgR > avg3R && avgR > redThreshold) {
                    position = RobotHardware.CenterStagePipeline.DetectionPosition.Left;
                }
                else if (avg2R > avgR && avg2R > avg3R && avg2R > redThreshold) {
                    position = RobotHardware.CenterStagePipeline.DetectionPosition.Middle;
                }
                else /*if (avg3R > avgR && avg3R > avg2R)* {
                    position = RobotHardware.CenterStagePipeline.DetectionPosition.Right;
                }
            }

    //blue
    //else {
    if (avgB > blueThreshold) {//value was 140
        color = CameraSubsystemTest.CameraPipeline.DetectionColor.Blue;
    }
    else if (avgR > redThreshold && avgB < yellowMinBlue) {
        color = CameraSubsystemTest.CameraPipeline.DetectionColor.Yellow;

    }
    else if(avgR > redThreshold)  {
        color = CameraSubsystemTest.CameraPipeline.DetectionColor.Red;
    }
    else{
        color = CameraSubsystemTest.CameraPipeline.DetectionColor.NONE;
    }
    // }


    return input;
}


public int getAnalysisLeft() {
    return avgB;
}

public int getAnalysisRight() {
    return avgR;
}
    }

    @Override
    public void periodic(){

    }*/
}