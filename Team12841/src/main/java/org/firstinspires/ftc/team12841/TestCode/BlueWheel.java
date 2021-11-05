package org.firstinspires.ftc.team12841.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team12841.Drivers.EncoderDrive;
import org.firstinspires.ftc.team12841.Drivers.RobotHardware;

import java.nio.file.Watchable;

@Autonomous(name = "blu wheel")

public class BlueWheel extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robotHardware = new RobotHardware(hardwareMap, this);
        EncoderDrive encoderDrive = new EncoderDrive((robotHardware));

        waitForStart();

        //move to wheel
        encoderDrive.StartAction(0.75, 30, 30, 5, true);
        sleep(200);
        encoderDrive.StartAction(0.75, 10, -10, 5, true);
        sleep(200);
        encoderDrive.StartAction(0.75, -39, -39, 5, true);
        sleep(200);

        //Below is the turntable spinning motor
        //robotHardware.WheelMotor.setPower(0.2);



        encoderDrive.StartAction(.75, 24, -24, 5, true);
        encoderDrive.StartAction(.75, -20, -20, 5, true);
    }
  }

