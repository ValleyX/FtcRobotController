package org.firstinspires.ftc.team2844.teleops;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.panels.Panels;
import com.pedropathing.math.Pose;
import com.vcs.valleylib.ftc.opmode.CommandOpMode;

import org.firstinspires.ftc.team2844.helpers.Constants;
import org.firstinspires.ftc.team2844.helpers.Subsystems;

public abstract class TeleOpBase extends CommandOpMode {
    protected Subsystems subsystems;
    protected TeleOpContainer container;
    public abstract int limelightPipeline();
    /**Constants.changeAlliance(alliance) MUST BE CALLED AS THE FIRST LINE OF TELEOP BECAUSE IT AFFECTS SETUP LOGIC*/
    public abstract int alliance();
    FieldManager field;

    @Override
    protected void initialize() {
        Constants.changeAlliance(alliance());
        subsystems = new Subsystems(hardwareMap, new Pose(0,0,0), limelightPipeline());
        container = new TeleOpContainer(subsystems, gamepad1, gamepad2);

        //setup the field on panels
        field = PanelsField.INSTANCE.getField();
        field.setBackground(PanelsField.INSTANCE.getImages().getBIOBUZZ().getDARK());
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    @Override
    protected void onStart(){
        subsystems.drive.startTeleOp();
        subsystems.light.startTimerLight();
    }

    @Override
    protected void run() {
        updatePanelsField();
        subsystems.update();

        //Telemetry goes here
    }

    private void updatePanelsField(){
        Pose p = subsystems.drive.getPose();
        field.setStyle(PanelsField.INSTANCE.getBLUE(), PanelsField.INSTANCE.getWHITE(), 1.0);
        field.moveCursor(p.x(), p.y());
        field.setCursorHeading(p.heading());
        field.circle(7.0);
        field.update();
    }

    @Override
    protected boolean enableCommandLogging() {
        return true;
    }
}
