package org.firstinspires.ftc.team12841.tests;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team12841.RobotHardware;

@TeleOp(name="DummyTeleOpTest", group="Test")
public class robotDummyTeleOpTest extends LinearOpMode {

    private Follower follower;

    @Override
    public void runOpMode() {
        // Initialize hardware
        RobotHardware robot = new RobotHardware(this);

        telemetry.addLine("Hardware Initialized");
        telemetry.update();

        // Wait for the game to start
        waitForStart();

        // Dummy loop: just keep reporting that hardware is alive
        while (opModeIsActive()) {
            telemetry.addLine("Robot running...");
            telemetry.addData("RF Motor Power", robot.rfMotor.getPower());
            telemetry.addData("RB Motor Power", robot.rbMotor.getPower());
            telemetry.addData("LF Motor Power", robot.lfMotor.getPower());
            telemetry.addData("LB Motor Power", robot.lbMotor.getPower());
            telemetry.update();
        }
    }
}
