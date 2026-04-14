package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Subsystems;

@TeleOp(name = "Test Voltage")
public class TestVoltage extends CommandOpMode {
    Subsystems subsystems;

    @Override
    public void initialize() {
        subsystems = new Subsystems(hardwareMap, 0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        while (opModeInInit()){
            subsystems.aimSubsystem.setPosition(0.0);
            telemetry.addData("Voltage At 0.0: ", subsystems.aimSubsystem.getVoltage());
            telemetry.addData("Axon degrees at 0.0: ", subsystems.aimSubsystem.getAxonValue());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()){
            subsystems.aimSubsystem.setPosition(1.0);
            telemetry.addData("Voltage At 1.0: ", subsystems.aimSubsystem.getVoltage());
            telemetry.addData("Axon degrees at 1.0: ", subsystems.aimSubsystem.getAxonValue());
            telemetry.update();
        }
    }
}
