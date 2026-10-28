package org.firstinspires.ftc.team2844.teleops.testing;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.pedropathing.math.Pose;
import com.vcs.valleylib.core.scheduler.CommandScheduler;
import com.vcs.valleylib.ftc.input.CommandGamepad;
import com.vcs.valleylib.ftc.opmode.CommandOpMode;

import org.firstinspires.ftc.team2844.helpers.Constants;
import org.firstinspires.ftc.team2844.helpers.PIDF;
import org.firstinspires.ftc.team2844.helpers.SavedVars;
import org.firstinspires.ftc.team2844.helpers.Subsystems;

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
        subsystems.light.startTimerLight();//starts the lights so they are accurate throughout the match

        //!Starts the state machine that controls everything, do not forget it!
        CommandScheduler.getInstance().schedule(container.getStateMachine());
    }

    @Override
    protected void run() {
        updatePanelsField();
        subsystems.update();

        //Telemetry
        telemetryBus.put("Use Bumpers to Switch States", "");
        telemetryBus.put("In State: ", container.getStateMachine().getState().toString());
        telemetryBus.put("", "");

        switch(container.getStateMachine().getState()){
            case DRIVE:
                telemetryBus.put("Dpad Down: ", "AutoPark");
                telemetryBus.put("Guide:     ", "Reset Pose");
                telemetryBus.put("Pose X: ", subsystems.drive.getPose().x());
                telemetryBus.put("Pose Y: ", subsystems.drive.getPose().y());
                telemetryBus.put("IMU:    ", subsystems.drive.getHeadingDegrees());
                break;

            case PIDF:
                telemetryBus.put("Triggers:            ", "Intake and Outtake");
                telemetryBus.put("Dpad Up and Down:    ", "Hood Control");
                telemetryBus.put("Dpad Left and Right: ", "Increase Velocity");
                telemetryBus.put("Left Stick Y:        ", "Selected PIDF");
                telemetryBus.put("Left Stick X:        ", "PIDF Multiplier");
                telemetryBus.put("Right Stick Y:       ", "Hood Multiplier");
                telemetryBus.put("Right Stick X:       ", "Velocity Multiplier");
                telemetryBus.put("A:                   ", "Change Selected Shooter");
                telemetryBus.put("Y:                   ", "Open/Close Gates");
                telemetryBus.put("X, B:                ", "Change Selected PIDF Value");
                telemetryBus.put("", "");
                telemetryBus.put("", "");
                telemetryBus.put("Selected Shooter:        ", container.currentShooter.toString());
                telemetryBus.put("Selected PIDF:           ", container.getSelectedPIDF());
                telemetryBus.put("Current PIDF Step Value: ", container.pidfArray[container.pidfMultiplier]);
                telemetryBus.put("", "");
                telemetryBus.put("Expected Velocity:           ", container.velocity);
                telemetryBus.put("Velocity:                    ", container.currentShooter.getVelocity());
                telemetryBus.put("Current Velocity Step Value: ", container.velocityArray[container.velocityMultiplier]);
                telemetryBus.put("", "");
                telemetryBus.put("Hood Pos:        ", container.currentShooter.getHood());
                telemetryBus.put("Hood Step Value: ", container.hoodArray[container.hoodMultiplier]);
                telemetryBus.put("", "");
                PIDF temp = container.currentShooter.getPIDF();
                telemetryBus.put("P:  ", temp.kP);
                telemetryBus.put("I:  ", temp.kI);
                telemetryBus.put("D:  ", temp.kD);
                telemetryBus.put("kS: ", temp.kS);
                telemetryBus.put("kV: ", temp.kV);
                break;

            case LINREG:
                telemetryBus.put("Triggers:            ", "Intake and Outtake");
                telemetryBus.put("Dpad Up and Down:    ", "Hood Control");
                telemetryBus.put("Dpad Left and Right: ", "Increase Velocity");
                telemetryBus.put("Right Stick Y:       ", "Hood Multiplier");
                telemetryBus.put("Right Stick X:       ", "Velocity Multiplier");
                telemetryBus.put("Y:                   ", "Open/Close Gates");
                telemetryBus.put("Velocity Multiplier: ", container.velocityArray[container.velocityMultiplier]);
                telemetryBus.put("Hood Multiplier:     ", container.hoodArray[container.hoodMultiplier]);
                telemetryBus.put("", "");
                telemetryBus.put("", "");
                telemetryBus.put("Expected Velocity:  ", container.velocity);
                telemetryBus.put("Nectar Velocity:    ", subsystems.nectar.getVelocity());
                telemetryBus.put("Pollen Velocity:    ", subsystems.pollen.getVelocity());
                telemetryBus.put("", "");
                telemetryBus.put("Hood Positions:     ", container.hoodPos);
                telemetryBus.put("", "");
                telemetryBus.put("Average Distance:   ", subsystems.distance());
                telemetryBus.put("Limelight Distance: ", subsystems.vision.getDistance());
                telemetryBus.put("Pinpoint Distance:  ", subsystems.drive.getDistance());

                break;

            case TURRET:
                telemetryBus.put("Right Stick X:       ", "Turret Multiplier");
                telemetryBus.put("X:                   ", "Turn Turret Left");
                telemetryBus.put("A:                   ", "Turn Turret Center");
                telemetryBus.put("B:                   ", "Turn Turret Right");
                telemetryBus.put("Y:                   ", "Track Turret");
                telemetryBus.put("Dpad Left and Right: ", "Step Turret");
                telemetryBus.put("", "");
                telemetryBus.put("", "");
                telemetryBus.put("Current Turret Relative Degrees:      ", subsystems.turret.getAbsoluteDegrees());
                telemetryBus.put("Current Field Centric Degrees:        ", subsystems.turret.getFieldCentricDegrees(subsystems.drive.getNormalHeadingDegrees()));
                telemetryBus.put("Expected Robot Tracking Degrees:      ", subsystems.turret.getRobotTargetDegrees());
                telemetryBus.put("Expected Field Tracking Degrees:      ", subsystems.turret.getFieldTargetDegrees());
                telemetryBus.put("Expected Turret Degrees(fed heading): ", subsystems.turret.getRobotTargetDegrees(subsystems.drive.getPose().x(), subsystems.drive.getPose().y(), subsystems.drive.getNormalHeadingDegrees()));
                telemetryBus.put("Turret Multiplier:                    ", container.turretArray[container.turretMultiplier]);
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
