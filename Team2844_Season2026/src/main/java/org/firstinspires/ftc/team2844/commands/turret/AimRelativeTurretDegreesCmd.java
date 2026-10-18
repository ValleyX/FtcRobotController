package org.firstinspires.ftc.team2844.commands.turret;

import com.vcs.valleylib.core.command.Command;

import org.firstinspires.ftc.team2844.helpers.Constants;
import org.firstinspires.ftc.team2844.subsystems.TurretSubsystem;

public class AimRelativeTurretDegreesCmd implements Command {
    TurretSubsystem turret;
    double degrees;
    public AimRelativeTurretDegreesCmd(TurretSubsystem turret, double degrees){
        this.turret = turret;
        this.degrees = degrees;
    }

    @Override
    public void execute() {
        turret.setRelativeDegrees(degrees);
    }

    @Override
    public boolean isFinished() {
        return thresholded(turret.getAbsoluteDegrees(), degrees);
    }

    private boolean thresholded(double currentDegrees, double desiredDegrees){
        return (desiredDegrees - Constants.TURRET_THRESHOLD_DEGREES < currentDegrees &&
                currentDegrees < desiredDegrees + Constants.TURRET_THRESHOLD_DEGREES);
    }
}
