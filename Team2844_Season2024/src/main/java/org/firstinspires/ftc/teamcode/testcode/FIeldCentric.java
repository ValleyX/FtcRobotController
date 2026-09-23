package org.firstinspires.ftc.team12841.testcode;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.command.CommandOpMode;

import com.arcrobotics.ftclib.command.button.Button;

//import org.firstinspires.ftc.teamcode.Drivers.RobotHardware;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.function.BooleanSupplier;

//TEST CODE
@TeleOp(name = "fieldcentricCommandTest")
public class FIeldCentric extends CommandOpMode {
    Button m_button;
    IMU newImu;
    public CameraSubsystemTest cameraSubsystemTest;
    GamepadEx driveGamepad;

    @Override
    public void initialize() /*throws InterruptedException*/ {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftRear");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightRear");

        //Servo servo =  hardwareMap.servo.get("servo");


        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        // Retrieve the IMU from the hardware map
        /*BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu2");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);*/

        IMU.Parameters newParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)
        );
        newImu = hardwareMap.get(IMU.class,"imu");
        newImu.initialize(newParameters);

        //WebcamName camCam = hardwareMap.get(WebcamName.class, "Webcamcolor");
        //CameraSubsystemTest.CameraPipeline pipeline = new CameraSubsystemTest.CameraPipeline(true);


        driveGamepad = new GamepadEx(gamepad1);


        FieldCentricTestDriveSubsystem drive = new FieldCentricTestDriveSubsystem(motorFrontLeft,motorFrontRight,motorBackRight,motorBackLeft,/*imu*/newImu);
        FieldCentricTestCommand driveCommand = new FieldCentricTestCommand(drive, this);
        AprilTagDriveSub aprilSubsystem = new AprilTagDriveSub(motorFrontLeft,motorFrontRight,motorBackRight,motorBackLeft);
        AprilTagTestCommand aprilTagTestCommand = new AprilTagTestCommand(aprilSubsystem,this);
        //cameraSubsystemTest = new CameraSubsystemTest(camCam,this,pipeline);
        ButtonTest buttonTest = new ButtonTest(drive);

        BooleanSupplier supplier = () -> gamepad1.left_bumper;
        //CameraSubsystemTest.CameraPipeline.DetectionColor Color = CameraSubsystemTest.CameraPipeline.DetectionColor.Yellow;

       /* while(opModeInInit()) {
            telemetry.addData("r value", cameraSubsystemTest.m_pipeline.avgR);
            telemetry.addData("b value", cameraSubsystemTest.m_pipeline.avgB);
            telemetry.addData("brick color", cameraSubsystemTest.m_pipeline.color);
            telemetry.update();//hello
       }

        cameraSubsystemTest.switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraSubsystemTest.cameraMonitorViewId, cameraSubsystemTest.m_camCam, cameraSubsystemTest.m_camCam);


        cameraSubsystemTest.switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                //pick desired camera here
                if (true) {
                    cameraSubsystemTest.switchableWebcam.setPipeline(cameraSubsystemTest.m_pipeline);
                    cameraSubsystemTest.switchableWebcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    cameraSubsystemTest.switchableWebcam.setActiveCamera(cameraSubsystemTest.m_camCam);

                } else {
                    cameraSubsystemTest.switchableWebcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    //switchableWebcam.setActiveCamera(webcamRight);
                    cameraSubsystemTest.switchableWebcam.setActiveCamera(cameraSubsystemTest.m_camCam);
                }


            }


            @Override
            public void onError(int errorCode) {
                ///
                //* This will be called if the camera could not be opened
                //
            }


        });
        waitForStart();

        telemetry.addData("r value", cameraSubsystemTest.m_pipeline.avgR);
        telemetry.addData("b value", cameraSubsystemTest.m_pipeline.avgB);
        telemetry.addData("brick color", cameraSubsystemTest.m_pipeline.color);
        telemetry.addData("Frame Count", cameraSubsystemTest.switchableWebcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", cameraSubsystemTest.switchableWebcam.getFps()));
        telemetry.addData("Total frame time ms", cameraSubsystemTest.switchableWebcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", cameraSubsystemTest.switchableWebcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", cameraSubsystemTest.switchableWebcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", cameraSubsystemTest.switchableWebcam.getCurrentPipelineMaxFps());*/



        /*m_button = new Button(supplier) {
            @Override
            public Button whileHeld(Command command, boolean interruptible) {
                return super.whileHeld(command, interruptible);
            }
        };

        m_button.whileHeld(buttonTest,true);*/
        m_button = (new GamepadButton(driveGamepad, GamepadKeys.Button.A)).whileHeld(buttonTest,true);

        //if(gamepad1.a){
       //     servo.setPosition(1);

       // }else{
       //     servo.setPosition(0);
       // }




       /* if (isStopRequested()) {


           cameraSubsystemTest.switchableWebcam.stopStreaming();
            return;

        }*/

        register(drive);
       drive.setDefaultCommand(driveCommand);
        telemetry.update();

        /*while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            telemetry.addData("rotX", rotX);
            telemetry.addData("rotY", rotY);
            telemetry.addData("front left power", frontLeftPower);
            telemetry.addData("front right power", frontRightPower);
            telemetry.addData("back left power", backLeftPower);
            telemetry.addData("back right power", backRightPower);
            telemetry.addData("denominator", denominator);
            telemetry.update();


        }*/

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();




        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
            /*telemetry.addData("r value", cameraSubsystemTest.m_pipeline.avgR);
            telemetry.addData("b value", cameraSubsystemTest.m_pipeline.avgB);
            telemetry.addData("brick color", cameraSubsystemTest.m_pipeline.color);
            telemetry.update();*/
        }
        reset();
    }


    public class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
        }


    }
}