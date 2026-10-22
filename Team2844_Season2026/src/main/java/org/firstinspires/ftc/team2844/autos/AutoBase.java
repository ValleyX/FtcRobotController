package org.firstinspires.ftc.team2844.autos;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.scheduler.CommandScheduler;
import com.vcs.valleylib.ftc.opmode.CommandOpMode;

import org.firstinspires.ftc.team2844.commands.drive.GoToCmd;
import org.firstinspires.ftc.team2844.commands.turret.DefaultTurretCmd;
import org.firstinspires.ftc.team2844.helpers.Constants;
import org.firstinspires.ftc.team2844.helpers.SavedVars;
import org.firstinspires.ftc.team2844.helpers.Subsystems;

public abstract class AutoBase extends CommandOpMode {
    protected Subsystems subsystems;
    DefaultTurretCmd turretCmd;
    public abstract Pose startingPose();
    public abstract int alliance();
    public abstract int target();
    protected abstract Command routine();
    protected PoseFactory field() {
        return (alliance() == Constants.BLUE_ALLIANCE) ? AutoPaths.BLUE : AutoPaths.RED;
    }

    @Override
    protected void initialize() {
        SavedVars.reset();
        int target = alliance()+target();
        Constants.changeAlliance(alliance());
        Constants.changeTarget(target);
        subsystems = new Subsystems(hardwareMap, startingPose(), target);

        //Keeps the turret tracking the entire auto
        turretCmd = new DefaultTurretCmd(subsystems.turret, subsystems.vision);
        subsystems.turret.setDefaultCommand(turretCmd);
        turretCmd.setTrack(true);
    }

    @Override
    protected void onStart() {
        CommandScheduler.getInstance().schedule(
            routine().withTimeout(25.0)
                    .andThen(() -> {
                        subsystems.nectar.idleShoot();
                        subsystems.pollen.idleShoot();
                        subsystems.intake.stopIntake();
                        subsystems.transfer.stop();
                        subsystems.transfer.closeGates();
                        turretCmd.setTrack(false);
                    })
                    .andThen(new GoToCmd(subsystems.drive,
                            ()->AutoPaths.routeToPark(field(), subsystems.drive.getPose())).withTimeout(4.75))
                    .finallyDo(this::handOffToTeleop)
        );
    }

    protected void handOffToTeleop(){
        Pose end = subsystems.drive.getPose();
        SavedVars.startingX = end.x();
        SavedVars.startingY = end.y();
        SavedVars.startingHeading = end.heading();
    }

    @Override
    protected void run() {
        subsystems.update();
    }
}
