package org.firstinspires.ftc.team12841.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.team12841.RobotHardware;
import org.firstinspires.ftc.team12841.pedroPathing.Constants;

import java.util.Arrays;
import java.util.List;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/*
 * This OpMode illustrates how to use the limelight Vision Sensor.
 *
 * @see <a href="https://limelightvision.io/">Limelight</a>
 *
 * Notes on configuration:
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */

        @TeleOp
        public class robotLimelightTest extends OpMode {
            // private Limelight3A limelight; //any limelight here
            public RobotHardware robotHardware = new RobotHardware(this);
            private Follower follower;
            private boolean following = false;
            private final Pose TARGET_LOCATION = new Pose(); //Put the target location here

            @Override
            public void init() {

                follower = Constants.createFollower(hardwareMap);
                follower.setStartingPose(new Pose()); //set your starting pose

                telemetry.setMsTransmissionInterval(11);

                robotHardware.limelight3A.pipelineSwitch(0);

            }

            @Override
            public void start() {
                robotHardware.limelight3A.start();
            }

            @Override
            public void loop() {
                follower.update();
                //if you're not using limelight you can follow the same steps: build an offset pose, put your heading offset, and generate a path etc
                if (!following) {
                    follower.followPath(
                            follower.pathBuilder()
                                    .addPath(new BezierLine(follower.getPose(), TARGET_LOCATION))
                                    .setLinearHeadingInterpolation(follower.getHeading(), TARGET_LOCATION.minus(follower.getPose()).getAsVector().getTheta())
                                    .build()
                    );
                }
                //This uses the aprilTag to relocalize your robot
                //You can also create a custom AprilTag fusion Localizer for the follower if you want to use this by default for all your autos
                follower.setPose(getRobotPoseFromCamera());
                if (following && !follower.isBusy()) following = false;
            }

            private Pose getRobotPoseFromCamera() {
                //Fill this out to get the robot Pose from the limelight's output (apply any filters if you need to using follower.getPose() for fusion)
                //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this

                    LLStatus status = robotHardware.limelight3A.getStatus();
                    telemetry.addData("Name", "%s",
                            status.getName());
                    telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                            status.getTemp(), status.getCpu(), (int) status.getFps());
                    telemetry.addData("Pipeline", "Index: %d, Type: %s",
                            status.getPipelineIndex(), status.getPipelineType());

                    LLResult result = robotHardware.limelight3A.getLatestResult();
                    if (result.isValid()) {
                        // Access general information
                        Pose3D botpose = result.getBotpose();
                        double captureLatency = result.getCaptureLatency();
                        double targetingLatency = result.getTargetingLatency();
                        double parseLatency = result.getParseLatency();
                        telemetry.addData("LL Latency", captureLatency + targetingLatency);
                        telemetry.addData("Parse Latency", parseLatency);
                        telemetry.addData("PythonOutput", Arrays.toString(result.getPythonOutput()));

                        telemetry.addData("tx", result.getTx());
                        telemetry.addData("txnc", result.getTxNC());
                        telemetry.addData("ty", result.getTy());
                        telemetry.addData("tync", result.getTyNC());

                        telemetry.addData("Botpose", botpose.toString());

                        // Access barcode results
                        List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                        for (LLResultTypes.BarcodeResult br : barcodeResults) {
                            telemetry.addData("Barcode", "Data: %s", br.getData());
                        }

                        // Access classifier results
                        List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                        for (LLResultTypes.ClassifierResult cr : classifierResults) {
                            telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                        }

                        // Access detector results
                        List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                        for (LLResultTypes.DetectorResult dr : detectorResults) {
                            telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                        }

                        // Access fiducial results
                        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                        for (LLResultTypes.FiducialResult fr : fiducialResults) {
                            telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                        }

                        // Access color results
                        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                        for (LLResultTypes.ColorResult cr : colorResults) {
                            telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                        }
                    } else {
                        telemetry.addData("Limelight", "No data available");
                    }

                    telemetry.update();


                robotHardware.limelight3A.stop();

                //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates
                return new Pose(0, 0, 0, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            }

        }