package org.firstinspires.ftc.team12841.Drivers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
//import org.firstinspires.ftc.team2844.RobotHardware;

import java.io.File;

public class OdometryDrive {
    //call the robot (to be readded when ready
    private RobotHardware robot_;

    //variables needed to help run program
    private ElapsedTime runtime_;
    private boolean waiting_;
    //Thread run condition
    private boolean isRunning = true;

    //odometry framework (to be built later as well possibly renamed
    public OdometryGlobalCoordinatePosition globalPositionUpdate;

    //creates thread
    public Thread positionThread;

    //Position variables used for storage and calculations
    double verticalRightEncoderWheelPosition = 0, verticalLeftEncoderWheelPosition = 0, normalEncoderWheelPosition = 0,  changeInRobotOrientation = 0;
    private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;
    private double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;

    //Algorithm constants
    private double robotEncoderWheelDistance;
    private double horizontalEncoderTickPerDegreeOffset;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    //Files to access the algorithm constants
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    //multipliers for the encoders
    private int verticalLeftEncoderPositionMultiplier = 1;
    private int verticalRightEncoderPositionMultiplier = 1;
    private int normalEncoderPositionMultiplier = 1;
    //OdometryGlobalCoordinatePosition globalPositionUpdate;
    //globalPositionUpdate =  new OdometryGlobalCoordinatePosition(robot_.leftVerticalEncoder, robot_.rightVerticalEncoder, robot_.horizontalEncoder, robot_.OD_COUNTS_PER_INCH, 75);

    //instatiate the method
    public OdometryDrive(RobotHardware robot ) {
        //set variables
        robot_ = robot;
        runtime_ = new ElapsedTime();
        waiting_ = false;


        //fill variables for the global position and the thread
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot_.verticalLeft, robot_.verticalRight, robot_.horizontal, robot_.OD_COUNTS_PER_INCH, 75);
        positionThread = new Thread(globalPositionUpdate);

        //start the thread
        positionThread.start();
    }

    //moves robot forward based on target and current position
    /* xTarget is the goal x position Assuming that the start position is (0,0) of  a graph. ytarget is the goal y position in a similar manner to the xtarget.
    iF YOU ARE GOING FORWARD OR BACKWARDS ONLY CHANGE THE YTARGET. if your x position has changed from the start the make sure that you use that new position if you
    change y target again. robotPower is how much power you are putting into the robot(WILL NOT EXCEED 0). desired robot orientation is what direction you want the robot facing
    (currently not implemented in any way SHOULD BE 0 FOR NOW). allowableDistanceError is how much error you want to allow your robot to be off by in inches.
     */
    public void goToPositionForward(double xTarget, double yTarget, double robotPower, double desiredRobotOrientation, double allowableDistanceError){
        //multiply distances by a constant
        xTarget *= robot_.OD_COUNTS_PER_INCH;
        yTarget *= robot_.OD_COUNTS_PER_INCH;
        allowableDistanceError *= robot_.OD_COUNTS_PER_INCH;

        //calculate how close we are to our target
        double distanceToX = xTarget - globalPositionUpdate.returnXCoordinate();
        double distanceToY = yTarget - globalPositionUpdate.returnYCoordinate();

        //calculate distance to final pos
        double distance = Math.hypot(distanceToX, distanceToY);

        //while not within tolerance of our target
        while (robot_.OpMode_.opModeIsActive() && Math.abs(distanceToY) > allowableDistanceError) {

            //update error values
            distanceToX = xTarget - globalPositionUpdate.returnXCoordinate();
            distanceToY = yTarget - globalPositionUpdate.returnYCoordinate();

            //calculate the angle we need to go
            double movementAngle = Math.toDegrees(Math.atan2(distanceToX, distanceToY));

            //get distance vectors for x and y
            double robotMovementX = calculateX(movementAngle, robotPower);
            double robotMovementY = calculateY(movementAngle, robotPower);

            //how much we need to corect our turn
            double turnCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();

            //update distance in loop
            distance = Math.hypot(distanceToX, distanceToY);


            //telemetry data
            robot_.OpMode_.telemetry.addData("robot movement x:", robotMovementX);
            robot_.OpMode_.telemetry.addData("robot movement y:", robotMovementY);
            robot_.OpMode_.telemetry.addData("X distance:", distanceToX);
            robot_.OpMode_.telemetry.addData("y distance:", distanceToY);
            robot_.OpMode_.telemetry.addData("movement angle", movementAngle);
            robot_.OpMode_.telemetry.addData("turn Correction:", turnCorrection);
            robot_.OpMode_.telemetry.addData("distance:", distance);
            robot_.OpMode_.telemetry.addData("glob x: ",globalPositionUpdate.returnXCoordinate());
            robot_.OpMode_.telemetry.addData("glob y: ",globalPositionUpdate.returnYCoordinate());
            robot_.OpMode_.telemetry.addData("glob orient: ",globalPositionUpdate.returnOrientation());
            robot_.OpMode_.telemetry.addData("encoder left: ", robot_.verticalLeft.getCurrentPosition());
            robot_.OpMode_.telemetry.addData("encoder right: ", robot_.verticalRight.getCurrentPosition());
            robot_.OpMode_.telemetry.addData("encoder horz: ", robot_.horizontal.getCurrentPosition());

            System.out.println("ValleyX: distanceToX " + distanceToX);
            System.out.println("ValleyX: distanceToY " + distanceToY);
            System.out.println("ValleyX: distance " + distance);
            System.out.println("ValleyX: turnCorrection " + turnCorrection);
            System.out.println("ValleyX: robotMovementY " + robotMovementY);

            //constant to help with correcting angle
            double c_nPi = .08;

            robot_.OpMode_.telemetry.update();
            // (don't add back) robot_.allpower(-robotMovementY);

            //checks which way we need to correct the direction
            if (turnCorrection > 0)
            {
                //move forward and correct slightly to the right
                robot_.leftPower(-robotMovementY - (c_nPi * Math.abs(turnCorrection)));
                robot_.rightPower(-robotMovementY);
            }
            else
            {
                //move forward and correct slightly to the left
                robot_.rightPower(-robotMovementY - (c_nPi * Math.abs(turnCorrection)));
                robot_.leftPower(-robotMovementY);
            }
            robot_.OpMode_.idle();
        }
        //stop robot
        robot_.allpower(0);

    }

    /* xTarget is the goal x position Assuming that the start position is (0,0) of  a graph. ytarget is the goal y position in a similar manner to the xtarget.
    iF YOU ARE GOING FORWARD OR BACKWARDS ONLY CHANGE THE xTARGET. if your y position has changed from the start the make sure that you use that new position if you
    change x target again. robotPower is how much power you are putting into the robot(WILL NOT EXCEED 0). desired robot orientation is what direction you want the robot facing
    (currently not implemented in any way SHOULD BE 0 FOR NOW). allowableDistanceError is how much error you want to allow your robot to be off by in inches.
     */
    public void goToPositionSide(double xTarget, double yTarget, double robotPower, double desiredRobotOrientation, double allowableDistanceError){
        //multiply distances by a constant
        xTarget *= robot_.OD_COUNTS_PER_INCH;
        yTarget *= robot_.OD_COUNTS_PER_INCH;
        allowableDistanceError *= robot_.OD_COUNTS_PER_INCH;

        //calculate how close we are to our target
        double distanceToX = xTarget - globalPositionUpdate.returnXCoordinate();
        double distanceToY = yTarget - globalPositionUpdate.returnYCoordinate();

        //calculate distance to final pos
        double distance = Math.hypot(distanceToX, distanceToY);

        //while not within tolerance of our target
        while (robot_.OpMode_.opModeIsActive() && Math.abs(distanceToX) > allowableDistanceError) {

            //update error values
            distanceToX = xTarget - globalPositionUpdate.returnXCoordinate();
            distanceToY = yTarget - globalPositionUpdate.returnYCoordinate();

            //calculate the angle we need to go
            double movementAngle = Math.toDegrees(Math.atan2(distanceToX, distanceToY));

            //get distance vectors for x and yv
            double robotMovementX = calculateX(movementAngle, robotPower);
            double robotMovementY = calculateY(movementAngle, robotPower);

            //how much we need to corect our turn
            double turnCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();

            //update distance
            distance = Math.hypot(distanceToX, distanceToY);

            //add telemetry
            robot_.OpMode_.telemetry.addData("robot movement x:", robotMovementX);
            robot_.OpMode_.telemetry.addData("robot movement y:", robotMovementY);
            robot_.OpMode_.telemetry.addData("X distance:", distanceToX);
            robot_.OpMode_.telemetry.addData("y distance:", distanceToY);
            robot_.OpMode_.telemetry.addData("movement angle", movementAngle);
            robot_.OpMode_.telemetry.addData("turn Correction:", turnCorrection);
            robot_.OpMode_.telemetry.addData("distance:", distance);
            robot_.OpMode_.telemetry.addData("glob x: ",globalPositionUpdate.returnXCoordinate());
            robot_.OpMode_.telemetry.addData("glob y: ",globalPositionUpdate.returnYCoordinate());
            robot_.OpMode_.telemetry.addData("glob orient: ",globalPositionUpdate.returnOrientation());
            robot_.OpMode_.telemetry.addData("encoder left: ", robot_.verticalLeft.getCurrentPosition());
            robot_.OpMode_.telemetry.addData("encoder right: ", robot_.verticalRight.getCurrentPosition());
            robot_.OpMode_.telemetry.addData("encoder horz: ", robot_.horizontal.getCurrentPosition());

            System.out.println("ValleyX: distanceToX " + distanceToX);
            System.out.println("ValleyX: distanceToY " + distanceToY);
            System.out.println("ValleyX: distance " + distance);
            System.out.println("ValleyX: turnCorrection " + turnCorrection);
            System.out.println("ValleyX: robotMovementY " + robotMovementY);

            //constant to help with turn correction
            double c_nPi = .08;

            robot_.OpMode_.telemetry.update();
            // dont add robot_.allpower(-robotMovementY);
            // dont add   if (turnCorrection > 0)
            // dont add   {
            // dont add robot_.leftPower(-robotMovementY - (c_nPi * Math.abs(turnCorrection)));
            //dont add robot_.rightPower(-robotMovementY);

            //move the robot while strafing while correcting the pos slightly
            robot_.leftBackDrive.setPower(robotMovementX - (c_nPi * Math.abs(turnCorrection)));
            robot_.leftFrontDrive.setPower(-robotMovementX + (c_nPi * Math.abs(turnCorrection)));
            robot_.rightBackDrive.setPower(-robotMovementX + (c_nPi * Math.abs(turnCorrection)));
            robot_.rightFrontDrive.setPower((robotMovementX - c_nPi * Math.abs(turnCorrection)));


            robot_.OpMode_.idle();
        }
        //stop robot
        robot_.allpower(0);

    }

    /*robot power is speed; desired robot orientation is what dirextion, error is tolerance.
    IF POWER IS .5 MINIMUMN TOLERANCE IS 3.25.
     */
    public void changeRobotOrientation (double robotPower, double desiredRobotOrientation, double allowableDistanceError){


        double turnCorrection = desiredRobotOrientation -globalPositionUpdate.returnOrientation();

        //while not within tolerance of our target
        while (robot_.OpMode_.opModeIsActive() && Math.abs(turnCorrection) > allowableDistanceError) {



            //how much we need to corect our turn
            turnCorrection = desiredRobotOrientation-globalPositionUpdate.returnOrientation();



            //telemetry data

            robot_.OpMode_.telemetry.addData("turn Correction:", turnCorrection);

            robot_.OpMode_.telemetry.addData("glob x: ",globalPositionUpdate.returnXCoordinate());
            robot_.OpMode_.telemetry.addData("glob y: ",globalPositionUpdate.returnYCoordinate());
            robot_.OpMode_.telemetry.addData("glob orient: ",globalPositionUpdate.returnOrientation());
            robot_.OpMode_.telemetry.addData("encoder left: ", robot_.verticalLeft.getCurrentPosition());
            robot_.OpMode_.telemetry.addData("encoder right: ", robot_.verticalRight.getCurrentPosition());
            robot_.OpMode_.telemetry.addData("encoder horz: ", robot_.horizontal.getCurrentPosition());



            //constant to help with correcting angle
            double c_nPi = .08;

            robot_.OpMode_.telemetry.update();
            // (don't add back) robot_.allpower(-robotMovementY);

            //checks which way we need to correct the direction
            if (desiredRobotOrientation > 0)
            {
                robot_.leftPower(-robotPower);
                robot_.rightPower(robotPower);
            }
            else
            {
                //move forward and correct slightly to the left
                robot_.leftPower(robotPower);
                robot_.rightPower(-robotPower);
            }
            robot_.OpMode_.idle();
            //robot_.OpMode_.sleep(100);
            System.out.println("ValleyX: ycoordinate" + globalPositionUpdate.returnYCoordinate());
        }
        //stop robot
        robot_.allpower(0);
        System.out.println("valleyX: ycoordinate" + globalPositionUpdate.returnYCoordinate());


        //RESETS THE IMU


    }

    public double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    public double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }


}
