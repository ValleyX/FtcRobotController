package org.firstinspires.ftc.team2844.teleops;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.panels.Panels;
import com.pedropathing.math.Pose;
import com.vcs.valleylib.ftc.opmode.CommandOpMode;

import org.firstinspires.ftc.team2844.helpers.Subsystems;

public abstract class TeleOpBase extends CommandOpMode {
    protected Subsystems subsystems;
    protected TeleOpContainer container;
    public abstract int limelightPipeline();
    FieldManager field;

    @Override
    protected void initialize() {
        subsystems = new Subsystems(hardwareMap, new Pose(0,0,0), limelightPipeline());
        container = new TeleOpContainer(subsystems, gamepad1, gamepad2);

        field = PanelsField.INSTANCE.getField();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getDEFAULT_FTC());
    }

    @Override
    protected void run() {
        updatePanelsField();
    }

    private void updatePanelsField(){
        Pose p = subsystems.drive.getPose();
        field.setStyle(PanelsField.INSTANCE.getBLUE(), PanelsField.INSTANCE.getWHITE(), 1.0);
        field.moveCursor(p.x(), p.y());
        field.circle(7.0);
        field.update();
    }

    @Override
    protected boolean enableCommandLogging() {
        return true;
    }
}
