package org.firstinspires.ftc.team2844.Team2844_Decode.AprilTag;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

//@Autonomous
//@TeleOp
@Autonomous(name="Robot: LimeLight Turn Encoder", group="Robot")
public class LimeLightEncoder extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor turnMotor = null;

    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  motorSpeed     = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 288.0;

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.2;     // Max turn speed to limit turn rate.
    static final double     HEADING_THRESHOLD       = 1.25 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable.

    private Limelight3A limelight;
    LLResult llResult;
    //private IMU imu;



    /*@Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch((1)); //april tag pipeline for red side
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

    }*/


    @Override
    public void runOpMode() {


    // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            turnMotor = hardwareMap.get(DcMotor.class, "yawMotor");
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch((0)); //april tag pipeline for red side
        }
        //Start Limelight when start is pressed
        limelight.start();

        //This section will run until stop or timeout
        while (opModeIsActive()) {
            llResult = limelight.getLatestResult();
            if(gamepad1.x) {
                turnYawToPos(.25, 90);
            }
            if (gamepad1.a) {
                turnYawToPos(.25, 180);
            }
            if(gamepad1.y){
                turnYawToPos(.25, turnMotor.getCurrentPosition() +llResult.getTx());
            }


            if (llResult != null && llResult.isValid()) {
                Pose3D botPose = llResult.getBotpose();  //pull in MT1 data
                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Tarea", llResult.getTa());
                telemetry.addData("Current Heading", getTurnMotorHeading());
                telemetry.addData("Current Encoder Tics", turnMotor.getCurrentPosition());
                telemetry.update();



                /*if(Math.abs(llResult.getTx()) >= 1.25) {
                    turnYawToPos(0.25, getTurnMotorHeading() - llResult.getTx());
                }*/
            }


        }


    }

    /*@Override
    public void start() {
        limelight.start();
    }

    //@Override
    //public void loop() {
        //this is for IMU integration with Limelight
        /*YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Tarea", llResult.getTa());

            // for NO IMU

        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();  //pull in MT1 data
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Tarea", llResult.getTa());

        }
    }*/

    public void turnYawToPos(double speed, double degrees){
        double ticPerDegree = COUNTS_PER_MOTOR_REV/360.0;

        int target = (int)((degrees*ticPerDegree)+0.5);
        turnMotor.setTargetPosition(target);

        turnMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turnMotor.setPower(speed);

        int currentPos = turnMotor.getCurrentPosition();
        while(!(currentPos <= target+3) && !(currentPos >= target-3)){
            currentPos = turnMotor.getCurrentPosition();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Tarea", llResult.getTa());
            telemetry.addData("Current Heading", getTurnMotorHeading());
            telemetry.addData("Current Encoder Tics", turnMotor.getCurrentPosition());
            telemetry.addData("in While", 0);
            telemetry.update();
        }
    }

    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            llResult = limelight.getLatestResult();

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);


        }
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getTurnMotorHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double turn) {
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        if (turn > 1.0)
        {
            turn /= turn;
        }

        turnMotor.setPower(turn);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {
        telemetry.addData("Tx", llResult.getTx());
        telemetry.addData("Ty", llResult.getTy());
        telemetry.addData("Tarea", llResult.getTa());

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getTurnMotorHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getTurnMotorHeading() {
        double degreePerTic = 360.0/COUNTS_PER_MOTOR_REV;
        return turnMotor.getCurrentPosition()*degreePerTic;
    }

}
