package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vcs.valleylib.core.command.Commands;
import com.vcs.valleylib.ftc.input.CommandGamepad;
import com.vcs.valleylib.ftc.opmode.CommandOpMode;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems.ShooterSubsystem;

/**
 * Flywheel feedforward and P tuner. Drivetrain stays out of it — this is a
 * bench opmode.
 *
 * <table>
 *   <tr><td>y</td><td>toggle between the high and low target</td></tr>
 *   <tr><td>b</td><td>cycle the adjustment step size</td></tr>
 *   <tr><td>dpad left / right</td><td>F</td></tr>
 *   <tr><td>dpad down / up</td><td>P</td></tr>
 * </table>
 *
 * <p>Procedure, unchanged from the notes on the old opmode: start at the low
 * target and raise F until the wheel gets close, then flip to the high target
 * and see whether it over- or undershoots. Adjust F until both targets have
 * about the same small error, shrinking the step size as you close in. Only then
 * bring in P, and tune it for the fastest settle between the two targets that
 * doesn't overshoot.
 */
@TeleOp(name = "CB Flywheel Tune", group = "QualBot Command")
public class FlywheelTuningTeleOp extends CommandOpMode {

    private static final double HIGH_VELOCITY = 4500;
    private static final double LOW_VELOCITY = 3000;
    private static final double[] STEP_SIZES = {10.0, 1.0, 0.1, 0.001, 0.0001};

    private ShooterSubsystem shooter;
    private CommandGamepad driver;

    private double targetVelocity = HIGH_VELOCITY;
    private double p = 0;
    private double f = 0;
    private int stepIndex = 1;

    @Override
    protected void initialize() {
        shooter = new ShooterSubsystem(hardwareMap);
        driver = CommandGamepad.forLogitechF310(gamepad1);
        shooter.setVelocityPIDF(p, 0, 0, f);
    }

    @Override
    protected void configureBindings() {
        driver.y().onTrue(Commands.runOnce(() ->
                targetVelocity = targetVelocity == HIGH_VELOCITY ? LOW_VELOCITY : HIGH_VELOCITY));

        driver.b().onTrue(Commands.runOnce(() ->
                stepIndex = (stepIndex + 1) % STEP_SIZES.length));

        driver.dpadLeft().onTrue(Commands.runOnce(() -> adjust(0, -step())));
        driver.dpadRight().onTrue(Commands.runOnce(() -> adjust(0, step())));
        driver.dpadDown().onTrue(Commands.runOnce(() -> adjust(-step(), 0)));
        driver.dpadUp().onTrue(Commands.runOnce(() -> adjust(step(), 0)));

        shooter.setDefaultCommand(shooter.run(() -> shooter.setRawVelocity(targetVelocity)));
    }

    private double step() {
        return STEP_SIZES[stepIndex];
    }

    private void adjust(double dp, double df) {
        p += dp;
        f += df;
        shooter.setVelocityPIDF(p, 0, 0, f);
    }

    @Override
    protected void run() {
        double actual = shooter.getRawVelocity();

        telemetryBus.put("Target velocity", targetVelocity);
        telemetryBus.put("Current velocity", actual);
        telemetryBus.put("Error", targetVelocity - actual);
        telemetryBus.put("P (dpad up/down)", p);
        telemetryBus.put("F (dpad left/right)", f);
        telemetryBus.put("Step size (b)", step());
    }
}
