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
        encoderDrive.StartAction(0.75, 16, 16, 5, true);
        sleep(200);
        encoderDrive.StartAction(0.75, 10, -10, 5, true);
        sleep(200);
        encoderDrive.StartAction(0.75, -17.5001, -17.5001, 5, true);
        sleep(200);
        // where we would code the wheel spin thing//

        encoderDrive.StartAction(.75, -19, 19, 5, true);
    }
}