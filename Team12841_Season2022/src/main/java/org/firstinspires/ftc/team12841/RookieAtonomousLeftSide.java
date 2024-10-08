package org.firstinspires.ftc.team12841;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.team12841.drivers.EncoderDrive_UnderthehoodStuff;
import org.firstinspires.ftc.team12841.drivers.LiftDriver_UnderthehoodStuff;
import org.firstinspires.ftc.team12841.drivers.RobotHardware;

@Autonomous(name = "RookieAutonomousLeftSide")
public class RookieAtonomousLeftSide extends LinearOpMode {

    RobotHardware robotHardware;
    EncoderDrive_UnderthehoodStuff encoderDrive;
    LiftDriver_UnderthehoodStuff liftdriver;
    public double sensorstop_distance = 3;
    public double sensorstop_backward = -1;

    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this);
        RobotGyroscope Gyro = new RobotGyroscope(robotHardware);
        liftdriver = new LiftDriver_UnderthehoodStuff(robotHardware);

        while (!opModeIsActive())
        {
            telemetry.addData("Cone Color", robotHardware.pipeline.markerPos);
            telemetry.addData("Heading", robotHardware.imu.isGyroCalibrated());
            telemetry.update();

        }

        if (robotHardware.pipeline.markerPos == RobotHardware.SkystoneDeterminationPipeline.MarkerPos.Red)  //Red
        {


            robotHardware.pinch.setPosition(robotHardware.closedPinch);
            sleep(1000);
            liftdriver.moveInches(14,.3);
            sleep(500);
            Gyro.driveStraight(.3,3,0,sensorstop_distance); // drive up off the wall
            Gyro.turnToHeading(.3,41 /*screw you andrew*/,1500); // turn to smol junction
            Gyro.driveStraight(.3,-.5,38,sensorstop_backward); //back up
            liftdriver.moveInches(10, .3); // lower arm on pole
            sleep(500);
            robotHardware.pinch.setPosition(robotHardware.openPinch);
            liftdriver.moveInches(14,.3);
            //sleep(500);
            Gyro.turnToHeading(.3,0,1500); // turn back to going forward
            //sleep(500);
            liftdriver.moveInches(0,0.6); //lower arm
            sleep(500);

            Gyro.driveStraight(.5,61.5,0,-1); // push cone out of way
            Gyro.driveStraight(.3,-9,0,sensorstop_backward); // go back some


            //liftdriver.moveInches(5.5,.3);
            Gyro.turnToHeading(0.3, -90, 1500); // turn to face cone stack
            liftdriver.moveInches(5.3, .5); // lift arm to 5-cone stack
            Gyro.driveStraight(0.3, 21, -90,sensorstop_backward); // drive to cone stack
            /*robotHardware.pinch.setPosition(robotHardware.closedPinch); // grab cone
            sleep(500);

            Gyro.driveStraight(0.3, -1, -90,sensorstop_backward);
            liftdriver.moveInches(14,0.5); // lift cone to smol junction
            Gyro.driveStraight(0.3,-21,-90,sensorstop_backward); // back up
            Gyro.turnToHeading(0.2,-128, 1500); // turn to smol boi
            liftdriver.moveInches(10, .3); // lower arm onto pole
            sleep(500);
            robotHardware.pinch.setPosition(robotHardware.openPinch); // release cone
            //sleep(500);
            liftdriver.moveInches(15,.05);

            Gyro.turnToHeading(0.2,-90, 1500); // face cone stack
            liftdriver.moveInches(4,.5); // lift to 4-cone stack
            Gyro.driveStraight(0.3,24,-80,sensorstop_distance); // drive to cone stack*/
            robotHardware.pinch.setPosition(robotHardware.closedPinch); // grab cone
            sleep(500);

            Gyro.driveStraight(0.3, -1, -90,sensorstop_backward);
            liftdriver.moveInches(22, 0.5); // raise cone to mid junction height
            Gyro.driveStraight(0.3, -19, -90,sensorstop_backward); // back up to mid junction
            Gyro.turnToHeading(0.3, -218, 4000); // turn to face mid junction
            //sleep(500);
            Gyro.driveStraight(0.3, 1.25, -217,sensorstop_backward); // get closer to pole
            liftdriver.moveInches(16, .3); // lower arm onto pole
            sleep(500);
            robotHardware.pinch.setPosition(robotHardware.openPinch); // release cone yeetus the fetus
            //sleep(500);
            liftdriver.moveInches(22,.3);
            Gyro.turnToHeading(0.3,-90,3000);
            liftdriver.moveInches(3.5, 0.7);
            //sleep(1000);
            Gyro.driveStraight(0.3, 24, -90, sensorstop_backward);

            robotHardware.pinch.setPosition(robotHardware.closedPinch);//grab for big pole
            sleep(1000);
            Gyro.driveStraight(0.3, -1, -90, sensorstop_backward);
            liftdriver.moveInches(22, 0.3); // raise cone to mid junction height
            Gyro.driveStraight(0.3, -20, -90, sensorstop_backward); // back up to center
            Gyro.turnToHeading(0.3,-132,1500);//turn to face the big one backwards
            Gyro.driveStraight(0.3,-10,-130,-1); // back towards the big one
            liftdriver.moveInches(40.5,0.3); //move up the height
            sleep(2000);
            robotHardware.pinch.setPosition(robotHardware.openPinch);
            sleep(1000);
            liftdriver.moveInches(0,0.4); //gets back to floor S P E E D
            //sleep(100);
            liftdriver.moveInches(0,0.4); //gets back to floor S P E E D
            sleep(500);

            Gyro.driveStraight(0.3,8,-10,sensorstop_backward);
            Gyro.turnToHeading(0.3,-90,3000);
            Gyro.driveStraight(.5,23,-90,sensorstop_backward);

        }
        else if (robotHardware.pipeline.markerPos == RobotHardware.SkystoneDeterminationPipeline.MarkerPos.Green)     //Green
        {

            robotHardware.pinch.setPosition(robotHardware.closedPinch);
            sleep(1000);
            liftdriver.moveInches(14,.3);
            sleep(500);
            Gyro.driveStraight(.3,3,0,sensorstop_distance); // drive up off the wall
            Gyro.turnToHeading(.3,41 /*screw you andrew*/,1500); // turn to smol junction
            Gyro.driveStraight(.3,-.5,38,sensorstop_backward); //back up
            liftdriver.moveInches(10, .3); // lower arm on pole
            sleep(500);
            robotHardware.pinch.setPosition(robotHardware.openPinch);
            liftdriver.moveInches(14,.3);
            //sleep(500);
            Gyro.turnToHeading(.3,0,1500); // turn back to going forward
            //sleep(500);
            liftdriver.moveInches(0,0.6); //lower arm
            sleep(500);

            Gyro.driveStraight(.5,61.5,0,-1); // push cone out of way
            Gyro.driveStraight(.3,-9,0,sensorstop_backward); // go back some


            //liftdriver.moveInches(5.5,.3);
            Gyro.turnToHeading(0.3, -90, 1500); // turn to face cone stack
            liftdriver.moveInches(5.3, .5); // lift arm to 5-cone stack
            Gyro.driveStraight(0.3, 21, -90,sensorstop_backward); // drive to cone stack
            /*robotHardware.pinch.setPosition(robotHardware.closedPinch); // grab cone
            sleep(500);

            Gyro.driveStraight(0.3, -1, -90,sensorstop_backward);
            liftdriver.moveInches(14,0.5); // lift cone to smol junction
            Gyro.driveStraight(0.3,-21,-90,sensorstop_backward); // back up
            Gyro.turnToHeading(0.2,-128, 1500); // turn to smol boi
            liftdriver.moveInches(10, .3); // lower arm onto pole
            sleep(500);
            robotHardware.pinch.setPosition(robotHardware.openPinch); // release cone
            //sleep(500);
            liftdriver.moveInches(15,.05);

            Gyro.turnToHeading(0.2,-90, 1500); // face cone stack
            liftdriver.moveInches(4,.5); // lift to 4-cone stack
            Gyro.driveStraight(0.3,24,-80,sensorstop_distance); // drive to cone stack*/
            robotHardware.pinch.setPosition(robotHardware.closedPinch); // grab cone
            sleep(500);

            Gyro.driveStraight(0.3, -1, -90,sensorstop_backward);
            liftdriver.moveInches(22, 0.5); // raise cone to mid junction height
            Gyro.driveStraight(0.3, -19, -90,sensorstop_backward); // back up to mid junction
            Gyro.turnToHeading(0.3, -218, 4000); // turn to face mid junction
            //sleep(500);
            Gyro.driveStraight(0.3, 1.25, -217,sensorstop_backward); // get closer to pole
            liftdriver.moveInches(16, .3); // lower arm onto pole
            sleep(500);
            robotHardware.pinch.setPosition(robotHardware.openPinch); // release cone yeetus the fetus
            //sleep(500);
            liftdriver.moveInches(22,.3);
            Gyro.turnToHeading(0.3,-90,3000);
            liftdriver.moveInches(3.5, 0.7);
            //sleep(1000);
            Gyro.driveStraight(0.3, 24, -90, sensorstop_backward);

            robotHardware.pinch.setPosition(robotHardware.closedPinch);//grab for big pole
            sleep(1000);
            Gyro.driveStraight(0.3, -1, -90, sensorstop_backward);
            liftdriver.moveInches(22, 0.3); // raise cone to mid junction height
            Gyro.driveStraight(0.3, -20, -90, sensorstop_backward); // back up to center
            Gyro.turnToHeading(0.3,-127,1500);//turn to face the big one backwards
            Gyro.driveStraight(0.3,-10,-127,-1); // back towards the big one
            liftdriver.moveInches(40.5,0.3); //move up the height
            sleep(2000);
            robotHardware.pinch.setPosition(robotHardware.openPinch);
            sleep(1000);
            liftdriver.moveInches(0,0.4); //gets back to floor S P E E D
            //sleep(100);
            liftdriver.moveInches(0,0.4); //gets back to floor S P E E D
            sleep(500);

            Gyro.driveStraight(0.3,10,-10,sensorstop_backward);
            Gyro.turnToHeading(0.3,-90,3000);
            sleep(3000);
            //Gyro.driveStraight(.5,-25,-90,sensorstop_backward);

        }
        else //Blue
        {

            robotHardware.pinch.setPosition(robotHardware.closedPinch);
            sleep(1000);
            liftdriver.moveInches(14,.3);
            sleep(500);
            Gyro.driveStraight(.3,3,0,sensorstop_distance); // drive up off the wall
            Gyro.turnToHeading(.3,41 /*screw you andrew*/,1500); // turn to smol junction
            Gyro.driveStraight(.3,-.5,38,sensorstop_backward); //back up
            liftdriver.moveInches(10, .3); // lower arm on pole
            sleep(500);
            robotHardware.pinch.setPosition(robotHardware.openPinch);
            liftdriver.moveInches(14,.3);
            //sleep(500);
            Gyro.turnToHeading(.3,0,1500); // turn back to going forward
            //sleep(500);
            liftdriver.moveInches(0,0.6); //lower arm
            sleep(500);

            Gyro.driveStraight(.5,61.5,0,-1); // push cone out of way
            Gyro.driveStraight(.3,-9,0,sensorstop_backward); // go back some


            //liftdriver.moveInches(5.5,.3);
            Gyro.turnToHeading(0.3, -90, 1500); // turn to face cone stack
            liftdriver.moveInches(5.3, .5); // lift arm to 5-cone stack
            Gyro.driveStraight(0.3, 21, -90,sensorstop_backward); // drive to cone stack
            /*robotHardware.pinch.setPosition(robotHardware.closedPinch); // grab cone
            sleep(500);

            Gyro.driveStraight(0.3, -1, -90,sensorstop_backward);
            liftdriver.moveInches(14,0.5); // lift cone to smol junction
            Gyro.driveStraight(0.3,-21,-90,sensorstop_backward); // back up
            Gyro.turnToHeading(0.2,-128, 1500); // turn to smol boi
            liftdriver.moveInches(10, .3); // lower arm onto pole
            sleep(500);
            robotHardware.pinch.setPosition(robotHardware.openPinch); // release cone
            //sleep(500);
            liftdriver.moveInches(15,.05);

            Gyro.turnToHeading(0.2,-90, 1500); // face cone stack
            liftdriver.moveInches(4,.5); // lift to 4-cone stack
            Gyro.driveStraight(0.3,24,-80,sensorstop_distance); // drive to cone stack*/
            robotHardware.pinch.setPosition(robotHardware.closedPinch); // grab cone
            sleep(500);

            Gyro.driveStraight(0.3, -1, -90,sensorstop_backward);
            liftdriver.moveInches(22, 0.5); // raise cone to mid junction height
            Gyro.driveStraight(0.3, -19, -90,sensorstop_backward); // back up to mid junction
            Gyro.turnToHeading(0.3, -218, 4000); // turn to face mid junction
            //sleep(500);
            Gyro.driveStraight(0.3, 1.25, -217,sensorstop_backward); // get closer to pole
            liftdriver.moveInches(16, .3); // lower arm onto pole
            sleep(500);
            robotHardware.pinch.setPosition(robotHardware.openPinch); // release cone yeetus the fetus
            //sleep(500);
            liftdriver.moveInches(22,.3);
            Gyro.turnToHeading(0.3,-90,3000);
            liftdriver.moveInches(3.5, 0.7);
            //sleep(1000);
            Gyro.driveStraight(0.3, 24, -90, sensorstop_backward);

            robotHardware.pinch.setPosition(robotHardware.closedPinch);//grab for big pole
            sleep(1000);
            Gyro.driveStraight(0.3, -1, -90, sensorstop_backward);
            liftdriver.moveInches(22, 0.3); // raise cone to mid junction height
            Gyro.driveStraight(0.3, -20, -90, sensorstop_backward); // back up to center
            Gyro.turnToHeading(0.3,-132,1500);//turn to face the big one backwards
            Gyro.driveStraight(0.3,-10,-130,-1); // back towards the big one
            liftdriver.moveInches(40.5,0.3); //move up the height
            sleep(2000);
            robotHardware.pinch.setPosition(robotHardware.openPinch);
            sleep(1000);
            liftdriver.moveInches(0,0.4); //gets back to floor S P E E D
            //sleep(100);
            liftdriver.moveInches(0,0.4); //gets back to floor S P E E D
            sleep(500);

            Gyro.driveStraight(0.3,8,-10,sensorstop_backward);
            Gyro.turnToHeading(0.3,-90,3000);
            Gyro.driveStraight(.5,-25,-90,sensorstop_backward);






           /* robotHardware.pinch.setPosition(robotHardware.closedPinch);
            sleep(1000);
            liftdriver.moveInches(14,.3);
            sleep(500);
            Gyro.driveStraight(.3,3,0,sensorstop_distance); // drive up off the wall
            Gyro.turnToHeading(.3,41 /*screw you andrew/,1500); // turn to smol junction
            Gyro.driveStraight(.3,-.5,38,sensorstop_backward); //back up
            liftdriver.moveInches(10, .3); // lower arm on pole
            sleep(500);
            robotHardware.pinch.setPosition(robotHardware.openPinch);
            liftdriver.moveInches(14,.3);
            //sleep(500);
            Gyro.turnToHeading(.3,0,1500); // turn back to going forward
            //sleep(500);
            liftdriver.moveInches(0,0.6); //lower arm
            sleep(500);

            Gyro.driveStraight(.4,61.5,0,-1); // push cone out of way
            Gyro.driveStraight(.3,-9,0,sensorstop_backward); // go back some


            //liftdriver.moveInches(5.5,.3);
            Gyro.turnToHeading(0.3, -90, 1500); // turn to face cone stack
            liftdriver.moveInches(5.3, .5); // lift arm to 5-cone stack
            Gyro.driveStraight(0.3, 21, -90,sensorstop_distance); // drive to cone stack
            robotHardware.pinch.setPosition(robotHardware.closedPinch); // grab cone
            sleep(500);

            Gyro.driveStraight(0.3, -1, -90,sensorstop_backward);
            liftdriver.moveInches(14,0.5); // lift cone to smol junction
            Gyro.driveStraight(0.3,-21,-90,sensorstop_backward); // back up
            Gyro.turnToHeading(0.2,-128, 1500); // turn to smol boi
            liftdriver.moveInches(10, .3); // lower arm onto pole
            sleep(500);
            robotHardware.pinch.setPosition(robotHardware.openPinch); // release cone
            //sleep(500);
            liftdriver.moveInches(15,.05);

            Gyro.turnToHeading(0.2,-90, 1500); // face cone stack
            liftdriver.moveInches(4,.5); // lift to 4-cone stack
            Gyro.driveStraight(0.3,24,-90,sensorstop_distance); // drive to cone stack
            robotHardware.pinch.setPosition(robotHardware.closedPinch); // grab cone
            sleep(500);

            Gyro.driveStraight(0.3, -1, -90,sensorstop_backward);
            liftdriver.moveInches(22, 0.5); // raise cone to mid junction height
            Gyro.driveStraight(0.3, -47.5, -90,sensorstop_backward); // back up to mid junction
            Gyro.turnToHeading(0.3, -125, 4000); // turn to face mid junction
            //sleep(500);
            Gyro.driveStraight(0.3, 1.5, -220,sensorstop_backward); // get closer to pole
            liftdriver.moveInches(17, .3); // lower arm onto pole
            sleep(500);
            robotHardware.pinch.setPosition(robotHardware.openPinch); // release cone
            //sleep(500);
            liftdriver.moveInches(22,.3);
            Gyro.turnToHeading(0.3,-90,3000);
            liftdriver.moveInches(0,0.5);
            sleep(3000); */
        }
    }
}