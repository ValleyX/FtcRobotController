package org.firstinspires.ftc.teamcode.testcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drivers.OdometryDrive;
import org.firstinspires.ftc.teamcode.Drivers.RobotHardware;

@Autonomous(name="OdometryTest")
public class OdTest extends LinearOpMode {

    // OdometryGlobalCoordinatePosition globalPositionUpdate;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardwareTestVersion robot = new RobotHardwareTestVersion(this,true);
        OdometryTestDrive odometryDrive = new OdometryTestDrive(robot);

        waitForStart();
        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        // globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.leftVerticalEncoder, robot.rightVerticalEncoder, robot.horizontalEncoder, robot.OD_COUNTS_PER_INCH, 75);
        // Thread positionThread = new Thread(globalPositionUpdate);
        // positionThread.start();

        telemetry.addData("Thread Active", odometryDrive.positionThread.isAlive());
        telemetry.update();


        //odometryDrive.goToPositionForward(0,10,.5,0,1);
        sleep(1000);
        odometryDrive.goToPositionSide(-10,0,.5,0,1);
        //  while (opModeIsActive());
        //   globalPositionUpdate.stop();


    }
}