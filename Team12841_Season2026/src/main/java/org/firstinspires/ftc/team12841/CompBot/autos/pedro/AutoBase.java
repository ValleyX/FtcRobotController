package org.firstinspires.ftc.team12841.CompBot.autos.pedro;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.scheduler.CommandScheduler;
import com.vcs.valleylib.ftc.opmode.CommandOpMode;

import org.firstinspires.ftc.team12841.CompBot.Constants;
import org.firstinspires.ftc.team12841.CompBot.RobotHardware;

public abstract class AutoBase extends CommandOpMode {
    protected abstract int alliance();
    protected abstract Pose startingPose();
    protected abstract Command routine();

    protected PoseFactory field() {
        return (alliance() == Constants.BLUE_ALLIANCE) ? AutoPaths.BLUE : AutoPaths.RED;
    }


    protected RobotHardware robot;
    FieldManager panelsField;

    public AutoBase(){}

    @Override
    protected void initialize() {
        robot = new RobotHardware(this.hardwareMap, this.telemetry, ()->false, false);

        //setup the field on panels
        panelsField = PanelsField.INSTANCE.getField();
        panelsField.setBackground(PanelsField.INSTANCE.getImages().getBIOBUZZ().getDARK());
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    @Override
    protected void onStart() {
        CommandScheduler.getInstance().schedule(
                routine().withTimeout(29.75)
        );
    }

    @Override
    protected void run() {
        updatePanelsField();
        robot.updatePose();
        //Where the robot thinks it is
        Pose p = robot.getPose();
        //Telemetry Goes here
        telemetryBus.put("X ", p.x());
        telemetryBus.put("Y ", p.y());
        telemetryBus.put("Heading in degrees", Math.toDegrees(p.heading()));
    }

    /**Draws the robot (circle) and which way it is facing (line) on the Panels field*/
    private void updatePanelsField(){
        Pose p = robot.getPose();
        panelsField.setStyle(PanelsField.INSTANCE.getBLUE(), PanelsField.INSTANCE.getWHITE(), 1.0);
        panelsField.moveCursor(p.x(), p.y());
        panelsField.circle(7.0);
        panelsField.line(p.x() + 7.0 * Math.cos(p.heading()), p.y() + 7.0 * Math.sin(p.heading()));
        panelsField.update();
    }
}
