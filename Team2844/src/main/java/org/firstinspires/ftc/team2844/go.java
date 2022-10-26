package org.firstinspires.ftc.team2844;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team2844.drivers.EncoderDriveMecha;
import org.firstinspires.ftc.team2844.drivers.RobotAutoDriveByGyro_Linear;

@Autonomous(name="microphoneActivateTest")
public class go extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        EncoderDriveMecha mechaDrive = new EncoderDriveMecha(robot);
        RobotAutoDriveByGyro_Linear gyroTurn = new RobotAutoDriveByGyro_Linear(robot);
        waitForStart();

        sleep(5000);
        gyroTurn.turnToHeading(.8,180);
        gyroTurn.turnToHeading(.8,0);

        //robot.allpower(0);

    }
}
