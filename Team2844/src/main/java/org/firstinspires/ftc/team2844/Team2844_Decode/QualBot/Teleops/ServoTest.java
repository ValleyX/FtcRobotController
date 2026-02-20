package org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.RobotHardware;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.ShooterHardware;

@Disabled
@TeleOp(name = "TestServo")
public class ServoTest extends LinearOpMode {
    RobotHardware robotHardware;
    ShooterHardware shooterHardware;
    LinearOpMode OpMode_;
    boolean dPadLeft = false;
    boolean dPadRight = false;
    double blockPos = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this);
        shooterHardware = new ShooterHardware(this);
        OpMode_ = this;
        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.dpad_left) {
                if (!dPadLeft) {
                    blockPos -= 0.01;
                    dPadLeft = true;
                }
            } else {
                dPadLeft = false;
            }

            if (gamepad1.dpad_right) {
                if (!dPadRight) {
                    blockPos += 0.01;
                    dPadRight = true;
                }
            } else {
                dPadRight = false;
            }

            shooterHardware.testServo(blockPos);
            telemetry.addData("Block Pos", blockPos);
            telemetry.update();
        }
    }
}
