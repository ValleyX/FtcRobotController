package org.firstinspires.ftc.team12841.testcode;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//TESTCODE

import java.util.List;

public class AprilTagTestCommand extends CommandBase {
    private final AprilTagDriveSub m_driveSub;
    public LinearOpMode m_opMode;
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 12;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


    boolean targetFound = false;    // Set to true when an AprilTag target is detected
    double drive = 0;        // Desired forward power/speed (-1 to +1)
    double strafe = 0;        // Desired strafe power/speed (-1 to +1)
    double turn = 0;        // Desired turning power/speed (-1 to +1)
    public AprilTagTestCommand(AprilTagDriveSub driveSub,LinearOpMode opMode){
        m_driveSub = driveSub;
        m_opMode = opMode;
        addRequirements(m_driveSub);

    }

    @Override
    public void initialize(){

        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(m_opMode.hardwareMap.get(WebcamName.class, "Webcamcolor"))
                    .addProcessor(aprilTag)
                    .build();
        } /*else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }*/
    }

    @Override
    public void execute(){
        //reverses motors so that the FIRST provided code works correctly
        m_driveSub.m_frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        m_driveSub.m_backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        m_driveSub.m_frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        m_driveSub.m_backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        targetFound = false;
        desiredTag = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) &&
                    ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))) {
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            } else {
                m_opMode.telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            m_opMode.telemetry.addData(">", "HOLD Left-Bumper to Drive to Target\n");
            m_opMode.telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            m_opMode.telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
            m_opMode.telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            m_opMode.telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            m_opMode.telemetry.addData(">", "Drive using joysticks to find valid target\n");
        }

        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
        if (/*gamepad1.left_bumper &&*/ targetFound) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (desiredTag.ftcPose.range - m_driveSub.DESIRED_DISTANCE);
            double headingError = desiredTag.ftcPose.bearing;
            double yawError = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = Range.clip(rangeError * m_driveSub.SPEED_GAIN, -m_driveSub.MAX_AUTO_SPEED, m_driveSub.MAX_AUTO_SPEED);
            turn = Range.clip(headingError * m_driveSub.TURN_GAIN, -m_driveSub.MAX_AUTO_TURN, m_driveSub.MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * m_driveSub.STRAFE_GAIN, -m_driveSub.MAX_AUTO_STRAFE, m_driveSub.MAX_AUTO_STRAFE);

            m_opMode.telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }/* else {

            // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
            drive = -gamepad1.left_stick_y / 2.0;  // Reduce drive rate to 50%.
            strafe = -gamepad1.left_stick_x / 2.0;  // Reduce strafe rate to 50%.
            turn = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
            telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }*/
        m_opMode.telemetry.update();

        // Apply desired axes motions to the drivetrain.
        m_driveSub.moveRobot(drive, strafe, turn);
        m_opMode.sleep(10);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}