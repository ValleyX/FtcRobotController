package org.firstinspires.ftc.team12841.Drivers;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTag {

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera

    private VisionPortal visionPortal;               // Used to manage the video source.
    public AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    //private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    // RobotHardware robot;

    public AprilTag(RobotHardware robot_) {

    }

    public void initAprilTag(RobotHardware robot) {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(robot.OpMode_.hardwareMap.get(WebcamName.class, "Webcamcolor"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    public void    setManualExposure(RobotHardware robot,int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            robot.OpMode_.telemetry.addData("Camera", "Waiting");
            robot.OpMode_.telemetry.update();
            while (!robot.OpMode_.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                robot.OpMode_.sleep(20);
            }
            robot.OpMode_.telemetry.addData("Camera", "Ready");
            robot.OpMode_.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!robot.OpMode_.isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                robot.OpMode_.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            robot.OpMode_.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            robot.OpMode_.sleep(20);
        }
    }
    /*
            public Boolean isAprilTagDetected (int Desired_id) {

                Boolean targetFound = false;


                // Step through the list of detected tags and look for a matching tag
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    if ((detection.metadata != null) &&
                            ( (detection.id == Desired_id))  ){
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        //robot.OpMode_.telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
                    }
                }

                // Tell the driver what we see, and what to do.
                if (targetFound) {
                    /*
                    robot.OpMode_.telemetry.addData(">","HOLD Left-Bumper to Drive to Target\n");
                    robot.OpMode_.telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                    robot.OpMode_.telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                    robot.OpMode_.telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                    robot.OpMode_.telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);

                     */
  /*
            } else {
                robot.OpMode_.telemetry.addData(">","Target not found\n");
            }
            robot.OpMode_.telemetry.update();
            return targetFound;


        }
*/
    //returns april tag that is detected
    public AprilTagDetection aprilTagDetected(RobotHardware robot, int Desired_id){
        AprilTagDetection desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) &&
                    ( (detection.id == Desired_id))  ){
                desiredTag = detection;
                break;  // don't look any further.
            } else {
                robot.OpMode_.telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
            }
        }
        return desiredTag;
    }



}
