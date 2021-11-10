package org.firstinspires.ftc.team2844.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team2844.Drivers.DistanceDriverTest;
import org.firstinspires.ftc.team2844.Drivers.EncoderDriveMecha;
import org.firstinspires.ftc.team2844.Drivers.MechaImuDriver;
import org.firstinspires.ftc.team2844.Drivers.RobotHardware;

@Autonomous(name="BlueSpinnerSide")
public class BlueSpinnerTest2844 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, this, 0, 0, RobotHardware.cameraSelection.LEFT);
        EncoderDriveMecha encodermecha = new EncoderDriveMecha(robot);
        MechaImuDriver headingdrive = new MechaImuDriver(robot);
        DistanceDriverTest driveto = new DistanceDriverTest(robot, headingdrive);


        waitForStart();

        headingdrive.gyroDrive(1,10,0);
        //sleep(1000);
        headingdrive.gyroTurn(0.5,-30);
        //sleep(500);
        headingdrive.gyroDrive(1,13,-30);
        //sleep(500);
        headingdrive.gyroTurn(0.2,-50);
        //sleep(500);
        headingdrive.gyroDrive(1,-28,-50);
        //sleep(500);
        robot.StraifRight(1);
        sleep(2000);
        robot.StraifLeft(1);
        sleep(100);
        headingdrive.gyroDrive(1,5,0);

        /*
        headingdrive.gyroTurn(0.5,-15);
        headingdrive.gyroDrive(1,15,-15);


         */



    }




}
