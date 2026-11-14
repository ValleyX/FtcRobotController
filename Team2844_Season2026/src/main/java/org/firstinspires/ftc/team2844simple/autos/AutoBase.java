package org.firstinspires.ftc.team2844simple.autos;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.command.Commands;
import com.vcs.valleylib.core.scheduler.CommandScheduler;
import com.vcs.valleylib.ftc.opmode.CommandOpMode;

import org.firstinspires.ftc.team2844simple.commands.drive.GoToCmd;
import org.firstinspires.ftc.team2844simple.helpers.Constants;
import org.firstinspires.ftc.team2844simple.helpers.SavedVars;
import org.firstinspires.ftc.team2844simple.helpers.Subsystems;
public abstract class AutoBase extends CommandOpMode {
    protected Subsystems subsystems;
    public abstract Pose startingPose();
    public abstract int alliance();
    public abstract int target();
    protected abstract Command routine();
    FieldManager panelsField;
    protected PoseFactory field() {
        return (alliance() == Constants.BLUE_ALLIANCE) ? AutoPaths.BLUE : AutoPaths.RED;
    }

    @Override
    protected void initialize() {
        SavedVars.reset();
        int target = alliance()+target();
        Constants.changeAlliance(alliance());
        Constants.changeSide(target());
        subsystems = new Subsystems(hardwareMap, startingPose(), target);

        //setup the field on panels
        panelsField = PanelsField.INSTANCE.getField();
        panelsField.setBackground(PanelsField.INSTANCE.getImages().getBIOBUZZ().getDARK());
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    @Override
    protected void onStart() {
        CommandScheduler.getInstance().schedule(
            routine().withTimeout(20.0)
                    .andThen(Commands.runOnce(() -> {
                        subsystems.pollen.idleShoot();
                        subsystems.intake.stopIntake();
                        subsystems.transfer.stop();
                        subsystems.transfer.closeGates();
                    }))
                    .andThen(new GoToCmd(subsystems.drive,
                            ()->AutoPaths.routeToPark(field(), ()->subsystems.drive.getPose(), alliance())).withTimeout(9.75))
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
        updatePanelsField();
        subsystems.update();
        //Where the robot thinks it is
        Pose p = subsystems.drive.getPose();
        telemetryBus.put("X ", p.x());
        telemetryBus.put("Y ", p.y());
        telemetryBus.put("Heading in degrees", Math.toDegrees(p.heading()));
    }

    /**Draws the robot (circle) and which way it is facing (line) on the Panels field*/
    private void updatePanelsField(){
        Pose p = subsystems.drive.getPose();
        panelsField.setStyle(PanelsField.INSTANCE.getBLUE(), PanelsField.INSTANCE.getWHITE(), 1.0);
        panelsField.moveCursor(p.x(), p.y());
        panelsField.circle(7.0);
        panelsField.line(p.x() + 7.0 * Math.cos(p.heading()), p.y() + 7.0 * Math.sin(p.heading()));
        panelsField.update();
    }
}
