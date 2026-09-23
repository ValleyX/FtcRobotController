package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vcs.valleylib.core.command.Commands;
import com.vcs.valleylib.ftc.input.CommandGamepad;
import com.vcs.valleylib.ftc.opmode.CommandOpMode;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems.ShooterSubsystem;

/**
 * Nudges the feed gate servo a hundredth at a time so its open and closed
 * positions can be read off the telemetry and copied into
 * {@link org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.RobotConstants}.
 */
@TeleOp(name = "CB Servo Tune", group = "QualBot Command")
public class ServoTuningTeleOp extends CommandOpMode {

    private static final double STEP = 0.01;

    private ShooterSubsystem shooter;
    private CommandGamepad driver;

    private double position = 0.0;

    @Override
    protected void initialize() {
        shooter = new ShooterSubsystem(hardwareMap);
        driver = CommandGamepad.forLogitechF310(gamepad1);
    }

    @Override
    protected void configureBindings() {
        driver.dpadLeft().onTrue(Commands.runOnce(() -> position -= STEP));
        driver.dpadRight().onTrue(Commands.runOnce(() -> position += STEP));

        shooter.setDefaultCommand(shooter.run(() -> shooter.setGatePosition(position)));
    }

    @Override
    protected void run() {
        telemetryBus.put("Gate position (dpad left/right)", position);
    }
}
