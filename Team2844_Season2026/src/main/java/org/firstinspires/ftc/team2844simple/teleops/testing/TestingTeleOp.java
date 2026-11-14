package org.firstinspires.ftc.team2844simple.teleops.testing;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.pedropathing.math.Pose;
import com.vcs.valleylib.core.scheduler.CommandScheduler;
import com.vcs.valleylib.ftc.input.CommandGamepad;
import com.vcs.valleylib.ftc.opmode.CommandOpMode;

import org.firstinspires.ftc.team2844simple.helpers.Constants;
import org.firstinspires.ftc.team2844simple.helpers.PIDF;
import org.firstinspires.ftc.team2844simple.helpers.SavedVars;
import org.firstinspires.ftc.team2844simple.helpers.Subsystems;

public abstract class TestingTeleOp extends CommandOpMode {
    protected Subsystems subsystems;
    public Pose startPose;
    protected TestingContainer container;
    protected abstract int alliance();
    FieldManager field;


    @Override
    protected void initialize() {
        Constants.changeAlliance(alliance());
        int target = (Constants.side != Constants.NO_TARGET)? Constants.side+alliance(): alliance();
        Constants.changeTarget(target);
        startPose = new Pose(0,0,0);

        subsystems = new Subsystems(hardwareMap, startPose, target);
        container = new TestingContainer(subsystems, new CommandGamepad(gamepad1), new CommandGamepad(gamepad2));

        //set up the field on panels
        field = PanelsField.INSTANCE.getField();
        field.setBackground(PanelsField.INSTANCE.getImages().getBIOBUZZ().getDARK());
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    @Override
    protected void onStart() {
        subsystems.drive.startTeleOp(); //sets all the drive motors to 0 power for the start of teleop so that no auto paths accidentally roll over

        //!Starts the state machine that controls everything, do not forget it!
        CommandScheduler.getInstance().schedule(container.getStateMachine());
    }

    @Override
    protected void run() {
        updatePanelsField();
        subsystems.update();

        //Telemetry
        telemetryBus.put("Use Bumpers to Switch States", "");
        telemetryBus.put("In State ", container.getStateMachine().getState().toString());
        telemetryBus.put(" ", " ");

        switch(container.getStateMachine().getState()){
            case DRIVE:
                telemetryBus.put("Dpad Down .", "AutoPark");
                telemetryBus.put("Guide     .", "Reset Pose");
                telemetryBus.put("Pose X .", subsystems.drive.getPose().x());
                telemetryBus.put("Pose Y .", subsystems.drive.getPose().y());
                telemetryBus.put("IMU    .", subsystems.drive.getNormalHeadingDegrees());
                break;

            case PIDF:
                telemetryBus.put("Triggers            .", "Intake and Outtake");
                telemetryBus.put("Dpad Left and Right .", "Increase Velocity");
                telemetryBus.put("Left Stick Y        .", "Selected PIDF");
                telemetryBus.put("Left Stick X        .", "PIDF Multiplier");
                telemetryBus.put("Right Stick X       .", "Velocity Multiplier");
                telemetryBus.put("Y                   .", "Open/Close Gate");
                telemetryBus.put("X, B                .", "Change Selected PIDF Value");
                telemetryBus.put("--------------------.", "-");
                telemetryBus.put("--------------------.", "-");
                telemetryBus.put("Expected Velocity           .", container.velocity);
                telemetryBus.put("Velocity                    .", subsystems.pollen.getRPM());
                telemetryBus.put("Current Velocity Step Value .", container.velocityArray[container.velocityMultiplier]);
                telemetryBus.put("--------------------", "-");
                telemetryBus.put("Selected PIDF           .", container.getSelectedPIDF());
                telemetryBus.put("Current PIDF Step Value .", container.pidfArray[container.pidfMultiplier]);
                telemetryBus.put("Pollen Power:           .", subsystems.pollen.getPower());
                telemetryBus.put("Gates Up?", subsystems.transfer.areGatesOpen());
                telemetryBus.put("Gate 1 Pos", subsystems.transfer.getGate1Pos());
                telemetryBus.put("Gate 2 Pos", subsystems.transfer.getGate2Pos());
                PIDF temp = subsystems.pollen.getPIDF();
                telemetryBus.put("P  ", temp.kP);
                telemetryBus.put("I  ", temp.kI);
                telemetryBus.put("D  ", temp.kD);
                telemetryBus.put("kS ", temp.kS);
                telemetryBus.put("kV ", temp.kV);

                break;

            case SHOOTING:
                telemetryBus.put("Triggers            .", "Intake and Outtake");
                telemetryBus.put("Dpad Left and Right .", "Increase Velocity");
                telemetryBus.put("Right Stick X       .", "Velocity Multiplier");
                telemetryBus.put("Y                   .", "Open/Close Gate");
                telemetryBus.put("Velocity Multiplier .", container.velocityArray[container.velocityMultiplier]);
                telemetryBus.put("--------------------.", "-");
                telemetryBus.put("--------------------.", "-");
                telemetryBus.put("Expected Velocity  .", container.velocity);
                telemetryBus.put("Pollen RPM    .", subsystems.pollen.getRPM());
                telemetryBus.put("--------------------.", "-");
                telemetryBus.put("Average Distance   .", subsystems.distance());
                telemetryBus.put("Limelight Distance .", subsystems.vision.getDistance());
                telemetryBus.put("Pinpoint Distance  .", subsystems.drive.getDistance());
                telemetryBus.put("Gates Up?", subsystems.transfer.areGatesOpen());
                telemetryBus.put("Gate 1 Pos", subsystems.transfer.getGate1Pos());
                telemetryBus.put("Gate 2 Pos", subsystems.transfer.getGate2Pos());
                telemetryBus.put("Shooter Motor Current", subsystems.pollen.getCurrent());
                break;
        }
    }

    @Override
    public void stop() {
        SavedVars.reset();
        super.stop();
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
