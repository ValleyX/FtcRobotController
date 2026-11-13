package org.firstinspires.ftc.team2844.teleops;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.pedropathing.math.Pose;
import com.vcs.valleylib.core.scheduler.CommandScheduler;
import com.vcs.valleylib.ftc.opmode.CommandOpMode;

import org.firstinspires.ftc.team2844.helpers.Constants;
import org.firstinspires.ftc.team2844.helpers.SavedVars;
import org.firstinspires.ftc.team2844.helpers.Subsystems;

public abstract class TeleOpBase extends CommandOpMode {
    protected Subsystems subsystems;
    public Pose startPose;
    public Pose resetPose;
    protected TeleOpContainer container;
    /**Constants.changeAlliance(alliance) MUST BE CALLED AS THE FIRST LINE OF TELEOP BECAUSE IT AFFECTS SETUP LOGIC*/
    public abstract int alliance();
    FieldManager field;

    @Override
    protected void initialize() {
        Constants.changeAlliance(alliance());
        int target = (Constants.side != Constants.NO_TARGET)? Constants.side+alliance(): alliance();
        Constants.changeTarget(target);
        resetPose = (Constants.alliance == Constants.BLUE_ALLIANCE)
                ?new Pose(144.0-(Constants.ROBOT_LENGTH/2.0), Constants.ROBOT_WIDTH/2.0,Math.toRadians(180.0))
                :new Pose(Constants.ROBOT_LENGTH/2.0, (144-Constants.ROBOT_WIDTH/2.0), 0.0);


        //! Checks for valid saved poses and uses whatever it can get. If none are found, sets it to 0,0,0
        if(SavedVars.validPose()){
            startPose = new Pose(SavedVars.startingX, SavedVars.startingY, SavedVars.startingHeading);
        } else if(SavedVars.validPosition()){
            startPose = new Pose(SavedVars.startingX, SavedVars.startingY, resetPose.heading());
        } else if(SavedVars.validHeading()){
            startPose = new Pose(resetPose.x(), resetPose.y(), SavedVars.startingHeading);
        } else {
            startPose = new Pose(resetPose.x(), resetPose.y(), resetPose.heading());
        }

        subsystems = new Subsystems(hardwareMap, startPose, target);
        container = new TeleOpContainer(subsystems, gamepad1, gamepad2);


        //setup the field on panels
        field = PanelsField.INSTANCE.getField();
        field.setBackground(PanelsField.INSTANCE.getImages().getBIOBUZZ().getDARK());
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    @Override
    protected void onStart(){
        subsystems.drive.startTeleOp(); //sets all the drive motors to 0 power for the start of teleop so that no auto paths accidentally roll over
        subsystems.light.startTimerLight();//starts the lights so they are accurate throughout the match

        subsystems.intake.flowerRampUp();
        subsystems.pollen.setHoodDegrees(Constants.MIN_HOOD_DEGREE);
        //!Starts the statemachine that controls everything, do not forget it!
        CommandScheduler.getInstance().schedule(container.getStateMachine());
    }

    @Override
    protected void run() {
        updatePanelsField();
        subsystems.update();
        //Telemetry goes here
        telemetryBus.put("State         ", container.teleOpCycle.getState());
        telemetryBus.put("Aimed at      ",  subsystems.turret.getAbsoluteDegrees());
        telemetryBus.put("Aimed ", subsystems.turret.aimed());
        telemetryBus.put("Aimed at Zero ", subsystems.turret.aimedAt(0.0));
        telemetryBus.put("Servo Pos     ", subsystems.turret.axonValues());
        telemetryBus.put("RPM ", subsystems.pollen.getRPM());
        telemetryBus.put("At Velocity ", subsystems.pollen.atVelocity());
        telemetryBus.put("Field Centric Aim ", subsystems.turret.getCurrentFieldCentricDegrees());
        telemetryBus.put("Hood Pos      ", subsystems.pollen.getHood());
        telemetryBus.put("Hood Degree   ", subsystems.pollen.getCurrentHoodDegrees());
        telemetryBus.put("Best Hood Degree ", subsystems.pollen.getExpectedHoodDegrees());
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
