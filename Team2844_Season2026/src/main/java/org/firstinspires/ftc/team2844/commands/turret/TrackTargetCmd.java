package org.firstinspires.ftc.team2844.commands.turret;

import com.vcs.valleylib.core.command.Command;

import org.firstinspires.ftc.team2844.helpers.Constants;
import org.firstinspires.ftc.team2844.subsystems.TurretSubsystem;
import org.firstinspires.ftc.team2844.subsystems.VisionSubsystem;

public class TrackTargetCmd implements Command {
    TurretSubsystem turret;
    VisionSubsystem vision;
    public TrackTargetCmd(TurretSubsystem turret, VisionSubsystem vision){
        this.turret = turret;
        this.vision = vision;
    }

    @Override
    public void initialize() {
        turret.aimToTarget();
    }

    @Override
    public void execute() {
        if(vision.valid()){
            turret.setRelativeDegrees(vision.getTx());
        } else {
            turret.aimToTarget();
        }
    }

    @Override
    public boolean isFinished() {
        return (thresholded(turret.getAbsoluteDegrees(), turret.getTargetDegrees())  ||
                thresholded(turret.getAbsoluteDegrees(), vision.getTx()));
    }

    private boolean thresholded(double currentDegrees, double desiredDegrees){
        return (desiredDegrees - Constants.TURRET_THRESHOLD_DEGREES < currentDegrees &&
                currentDegrees < desiredDegrees + Constants.TURRET_THRESHOLD_DEGREES);
    }
}
