package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.intake;

import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.subsystem.Subsystem;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.RobotConstants;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems.ShooterSubsystem;

import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;

/**
 * The auto collecting mode: intake forward with the gate shut and the flywheel
 * driven gently backwards, so balls stack up in the robot instead of trickling
 * through into the shooter.
 *
 * <p>Both powers depend on battery state, which is sampled once when the command
 * starts. That mirrors the old autos, which re-read the Control Hub voltage
 * before each collection and picked one of two power pairs.
 *
 * <p>Runs until something else ends it — pair it with the pickup path.
 */
public class CollectCommand implements Command {

    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final BooleanSupplier highVoltage;

    private double intakePower;
    private double shooterPower;

    public CollectCommand(IntakeSubsystem intake, ShooterSubsystem shooter, BooleanSupplier highVoltage) {
        this.intake = intake;
        this.shooter = shooter;
        this.highVoltage = highVoltage;
    }

    @Override
    public void initialize() {
        boolean fresh = highVoltage.getAsBoolean();
        intakePower = fresh
                ? RobotConstants.AUTO_INTAKE_POWER_HIGH_VOLTAGE
                : RobotConstants.AUTO_INTAKE_POWER_LOW_VOLTAGE;
        shooterPower = fresh
                ? RobotConstants.AUTO_INTAKE_REVERSE_POWER_HIGH_VOLTAGE
                : RobotConstants.AUTO_INTAKE_REVERSE_POWER_LOW_VOLTAGE;

        shooter.closeGate();
        intake.holdBall();
    }

    @Override
    public void execute() {
        intake.intake(intakePower);
        shooter.setPower(shooterPower);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        shooter.stop();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        Set<Subsystem> requirements = new HashSet<>();
        requirements.add(intake);
        requirements.add(shooter);
        return requirements;
    }

    @Override
    public String getName() {
        return "Collect";
    }
}
