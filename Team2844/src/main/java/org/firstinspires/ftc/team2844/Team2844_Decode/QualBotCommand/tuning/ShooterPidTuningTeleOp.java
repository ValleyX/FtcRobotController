package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.PedroConstants;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.QualBotRobot;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.teleop.QualBotTeleOpBase;

/**
 * Drive the robot normally while a second driver tunes the shooter's velocity
 * PIDF from the field — the command-based replacement for the old
 * {@code PID Tuning} opmode.
 */
@TeleOp(name = "CB Shooter PIDF Tuning", group = "QualBot Command")
public class ShooterPidTuningTeleOp extends QualBotTeleOpBase {

    private ShooterPidTuningContainer tuningContainer;

    @Override
    protected int limelightPipeline() {
        return 0;
    }

    @Override
    protected void initialize() {
        robot = new QualBotRobot(hardwareMap, PedroConstants.ftcPose(0, 0, 0), limelightPipeline());
        tuningContainer = new ShooterPidTuningContainer(robot, gamepad1, gamepad2);
        container = tuningContainer;
    }

    @Override
    protected void run() {
        super.run();
        telemetryBus.put("Tuning P", tuningContainer.getP());
        telemetryBus.put("Tuning I", tuningContainer.getI());
        telemetryBus.put("Tuning D", tuningContainer.getD());
        telemetryBus.put("Tuning F", tuningContainer.getF());
    }
}
