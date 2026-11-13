package org.firstinspires.ftc.team2844simple.commands.drive;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.subsystem.Subsystem;

import org.firstinspires.ftc.team2844simple.helpers.Constants;
import org.firstinspires.ftc.team2844simple.subsystems.DriveSubsystem;

import java.util.Collections;
import java.util.Set;
import java.util.function.DoubleSupplier;

/**
 * Field-centric driving where the driver keeps control of translation and the robot turns itself
 * so its front (the shooter) faces the current target, Constants.targetX / targetY.
 *
 * <p>Unlike {@link TurnToCmd}, this does not use a Pedro hold, so x/y are never locked. It runs
 * until it is interrupted, then the drive's default command takes back over.
 *
 * <p>Turn power = AIM_KP * error + AIM_KD * (rate of change of error), plus AIM_KS in the direction
 * of the error when outside AIM_TOLERANCE_DEG, clipped to AIM_MAX_TURN. All gains are in Constants.
 */
public class TurnToTargetCmd implements Command {
    private final DriveSubsystem drive;
    private final DoubleSupplier forward, strafe;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastError;
    private boolean firstLoop;

    /**
     * @param forward driver forward input, +1 drives away from the driver station wall (same as the default drive command)
     * @param strafe  driver strafe input, +1 drives to the left (same as the default drive command)
     */
    public TurnToTargetCmd(DriveSubsystem drive, DoubleSupplier forward, DoubleSupplier strafe){
        this.drive = drive;
        this.forward = forward;
        this.strafe = strafe;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(drive);
    }

    @Override
    public void initialize() {
        firstLoop = true;
        timer.reset();
    }

    @Override
    public void execute() {
        double turn = 0.0;

        if(drive.hasTarget()) {
            double error = drive.getHeadingErrorToTarget();
            double dt = timer.seconds();
            timer.reset();

            double derivative = 0.0;
            if(!firstLoop && dt > 0.0){
                derivative = wrap(error - lastError) / dt;
            }
            lastError = error;
            firstLoop = false;

            turn = Constants.AIM_KP * error + Constants.AIM_KD * derivative;
            if(Math.abs(error) > Math.toRadians(Constants.AIM_TOLERANCE_DEG)){
                turn += Math.copySign(Constants.AIM_KS, error);
            }
            turn = Math.max(-Constants.AIM_MAX_TURN, Math.min(Constants.AIM_MAX_TURN, turn));
        } else {
            firstLoop = true;
        }

        drive.driveFieldCentricWithTurnPower(forward.getAsDouble(), strafe.getAsDouble(), turn);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}

    /** Keeps an angle difference in -PI to PI so the error jumping across +/-180 does not spike the D term. */
    private static double wrap(double radians){
        double temp = radians;
        while (temp > Math.PI) temp -= 2*Math.PI;
        while (temp < -Math.PI) temp += 2*Math.PI;
        return temp;
    }
}
