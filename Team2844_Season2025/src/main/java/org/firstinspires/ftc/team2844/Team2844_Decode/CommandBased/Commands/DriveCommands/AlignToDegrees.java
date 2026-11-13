package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;

import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

/**
 * Turns the robot until the Limelight's horizontal offset is centred.
 *
 * <p>Adds a turn contribution on top of whatever the driver is doing rather than
 * taking the drivetrain over, which is why it deliberately declares no
 * requirements: {@code DriveSubsystem} sums align power with drive power.
 */
public class AlignToDegrees extends CommandBase {
    /**
     * Gives up after this long.
     *
     * <p>Without a bound this command could never finish: when the Limelight
     * loses the tag, tx reads {@link Constants#NO_LL} and no tx value will ever
     * satisfy the centred test, so the command would sit scheduled forever.
     */
    private static final long TIMEOUT_MS = 1500;

    DriveSubsystem driveSubsystem;
    double speed;
    DoubleSupplier tx;

    private final ElapsedTime timer = new ElapsedTime();

    public AlignToDegrees(DriveSubsystem driveSubsystem, DoubleSupplier tx, double speed) {
        this.driveSubsystem = driveSubsystem;
        this.tx = tx;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        double tx = this.tx.getAsDouble();
        double power = 0.0;
        if(tx != Constants.NO_LL)
            power = -(tx * Constants.ALIGN_GAIN * speed);
        driveSubsystem.setAlignPower(-power, -power, power, power);
    }

    @Override
    public boolean isFinished() {
        if (timer.time(TimeUnit.MILLISECONDS) >= TIMEOUT_MS) {
            return true;
        }

        double tx = this.tx.getAsDouble();
        if (tx == Constants.NO_LL) {
            // No tag this frame. Hold (execute() commands zero turn power) and
            // let the timeout end things if it does not come back.
            return false;
        }
        return Math.abs(tx) < Constants.TURRET_THRESHHOLD;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setAlignPower(0.0,0.0,0.0,0.0);
    }
}
