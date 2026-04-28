package org.firstinspires.ftc.team2844.Team2844_Decode.SummerCamp.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team2844.Team2844_Decode.SummerCamp.Hardware.RobotHardware;

@TeleOp(name = "ExampleTeleOp")
public class ExampleTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robotHardware = new RobotHardware(this);

        waitForStart();
        while(opModeIsActive() && isStopRequested()){

            robotHardware.drive(gamepad1.left_stick_y);

            robotHardware.turn(gamepad1.left_stick_x);
        }
    }
}
