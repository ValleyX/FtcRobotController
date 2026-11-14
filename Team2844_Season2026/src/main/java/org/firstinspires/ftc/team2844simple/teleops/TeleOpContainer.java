package org.firstinspires.ftc.team2844simple.teleops;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.command.Commands;
import com.vcs.valleylib.core.command.InstantCommand;
import com.vcs.valleylib.core.fsm.StateMachine;
import com.vcs.valleylib.ftc.RobotContainer;
import com.vcs.valleylib.ftc.input.CommandGamepad;
import com.vcs.valleylib.ftc.input.Trigger;

import org.firstinspires.ftc.team2844simple.autos.AutoPaths;
import org.firstinspires.ftc.team2844simple.commands.CommandFactory;
import org.firstinspires.ftc.team2844simple.commands.drive.GoToCmd;
import org.firstinspires.ftc.team2844simple.commands.drive.TurnToCmd;
import org.firstinspires.ftc.team2844simple.commands.drive.TurnToTargetCmd;
import org.firstinspires.ftc.team2844simple.helpers.Constants;
import org.firstinspires.ftc.team2844simple.helpers.Subsystems;

public class TeleOpContainer extends RobotContainer {
    public enum Cycle {IDLE, INTAKE, PREPARE, SHOOT, OUTTAKE}

    Subsystems subsystems;

    /* ---------- Controller and Buttons ---------- */
    CommandGamepad driver;
    CommandGamepad operator;

    /*--- driver ---*/
    final Trigger shootButton;
    final Trigger intakeButton;
    final Trigger outtakeButton;
    final Trigger resetIMUButton;
    final Trigger resetPoseButton;
    final Trigger parkButton;
    final Trigger trackToggleButton;

    /*--- operator/oh crap ---*/
    final Trigger trackDisableButton;

    /*--- turn to target tracking ---*/
    /**Toggled by gamepad 1. Starts off, so the driver has to turn tracking on.*/
    boolean trackEnabled = false;
    /**Toggled by gamepad 2. While true, tracking is always off no matter what gamepad 1 does.*/
    boolean trackDisabled = false;


    StateMachine<Cycle> teleOpCycle;
    CommandFactory factory;

    final Pose resetPose;

    public TeleOpContainer(Subsystems subsystems, Gamepad gamepad1, Gamepad gamepad2) {
        this.subsystems = subsystems;
        factory = new CommandFactory(subsystems);
        resetPose = (Constants.alliance == Constants.BLUE_ALLIANCE)
                ?new Pose(144.0-(Constants.ROBOT_LENGTH/2.0), Constants.ROBOT_WIDTH/2.0, Math.toRadians(180.0))
                :new Pose(Constants.ROBOT_LENGTH/2.0, (144-Constants.ROBOT_WIDTH/2.0), Math.toRadians(0.0));

        this.driver = CommandGamepad.forLogitechF310(gamepad1);
        this.operator = CommandGamepad.forLogitechF310(gamepad2);

        /* ---------- CONTROLLER BINDINGS ---------- */
        // ----- DRIVER ----- //
        this.shootButton = driver.rightBumper();
        this.intakeButton = driver.rightTriggerButton(0.05); //runs the intake
        this.outtakeButton = driver.leftTriggerButton(0.05); //runs the intake backwards
        this.resetIMUButton = driver.guide();
        this.resetPoseButton = driver.start();
        this.parkButton = driver.dpadDown();
        this.trackToggleButton = driver.leftBumper(); //toggles turning to the target while shooting

        // ----- OPERATOR ----- //
        this.trackDisableButton = operator.rightBumper(); //toggles a kill switch that keeps target tracking off



        Trigger readyToShoot = shootButton
                        .and(subsystems.pollen::atVelocity);

        teleOpCycle = new StateMachine<>(Cycle.IDLE) //initial state
                /*--- states ---*/
                //?IDLE: close the gates then turn on the transfer
                .onEnter(Cycle.IDLE,
                    subsystems.transfer::closeGates)
                .state(Cycle.IDLE, () -> {
                    subsystems.intake.stopIntake();
                    subsystems.transfer.transfer();
                    subsystems.pollen.idleShoot();
                })
                //?INTAKE: turn on the intake
                .state(Cycle.INTAKE, subsystems.intake::intake)

                //?PREPARE: turn off transfer, open gates, aim the turret to the target, spinup both shooters
                .onEnter(Cycle.PREPARE, () -> {
                    subsystems.transfer.stop();
                    subsystems.intake.stopIntake();
                })
                .state(Cycle.PREPARE,
                    subsystems.pollen::shoot
                )
                //?SHOOT: transfers balls into shooter when at velocity(due to transition requirements)
                .state(Cycle.SHOOT, () -> {
                    if(subsystems.pollen.atVelocity()){
                        subsystems.transfer.openTransferGates();
                    } else {
                        subsystems.transfer.closeTransferGates();
                    }
                    subsystems.pollen.shoot();
                    subsystems.transfer.fastTransfer();
                    subsystems.intake.intake();
                })

                //?OUTTAKE: spits out the balls
                // Reached by force state on button press
                .state(Cycle.OUTTAKE, () ->{
                    subsystems.transfer.spitTransfer();
                    subsystems.intake.outtake();
                })

                /*--- transition buttons ---*/
                //?Intake
                //acts like an intake button with proper procedure
                .transition(Cycle.IDLE, Cycle.INTAKE, intakeButton)
                .transition(Cycle.INTAKE, Cycle.IDLE, intakeButton.negate())
                //?Shooter
                //Acts like a shoot button with proper procedure
                .transition(Cycle.PREPARE, Cycle.SHOOT, readyToShoot)
                .transition(Cycle.SHOOT, Cycle.PREPARE, readyToShoot.negate())

                /*--- fast tracks ---*/
                //Fast track to shooting/intaking
                .transition(Cycle.IDLE, Cycle.PREPARE, shootButton)
                .transition(Cycle.PREPARE, Cycle.IDLE, intakeButton);

        configureBindings();
    }

    @Override
    public void configureBindings() {
        configureDefaults();
        configureDriving();
        configureIntake();
        configureShooting();
        configureOperator();
    }

    @Override
    public Command getAutonomousCommand() {
        return null;
    }

    public void configureDefaults(){
        /*--- defaults ---*/
        //drive in fieldCentric
        subsystems.drive.setDefaultCommand(subsystems.drive.run(
                () -> subsystems.drive.driveFieldCentric(
                        -driver.leftY(),
                        -driver.leftX(),
                        -driver.rightX()
                )
        ));


        /*--- triggers ---*/
        //?Change Target Trigger
        //Always checks the position of the robot and changes the pipeline and target based on that
        Trigger audienceSide = new Trigger(
                () -> subsystems.drive.getPose().y() < 72);

        //when the robot goes to the audience side, set the pipeline and target
        audienceSide.onTrue(
                new InstantCommand(() ->subsystems.vision.setPipeline(Constants.alliance + Constants.AUDIENCE_SIDE))
                .andThen(() ->Constants.changeSide(Constants.AUDIENCE_SIDE)));
        //when the robot goes to the back side, set the pipeline and target
        audienceSide.onFalse(new InstantCommand(() ->subsystems.vision.setPipeline(Constants.alliance + Constants.BACK_SIDE))
                .andThen(() ->Constants.changeSide(Constants.BACK_SIDE)));

    }

    public void configureDriving(){
        resetIMUButton.onTrue(
                Commands.runOnce(subsystems.drive::resetHeading)
        );

        resetPoseButton.onTrue(
                Commands.runOnce(()->subsystems.drive.setPose(resetPose))
        );

        parkButton.whileTrue(
                new GoToCmd(subsystems.drive,
                        () ->AutoPaths.routeToPark(AutoPaths.getField(), subsystems.drive.getPose()), 1.0)
                        .andThen(Commands.run(() -> {}, subsystems.drive))
        );
    }

    public void configureIntake(){
        outtakeButton.onTrue(new InstantCommand(() ->{
            if(!teleOpCycle.isIn(Cycle.OUTTAKE)){
                teleOpCycle.forceState(Cycle.OUTTAKE);
            }
        }));

        outtakeButton.onFalse(new InstantCommand(()->{
            if(teleOpCycle.isIn(Cycle.OUTTAKE)){
                Cycle back = teleOpCycle.getPreviousState();
                teleOpCycle.forceState((back == null || back == Cycle.OUTTAKE)? Cycle.IDLE: back);
            }
        }));
    }

    public void configureShooting(){
        //?Turn to target while driving
        //gamepad 1 toggles tracking on and off
        trackToggleButton.onTrue(new InstantCommand(() -> trackEnabled = !trackEnabled));
        //gamepad 2 toggles a kill switch that keeps tracking off
        trackDisableButton.onTrue(new InstantCommand(() -> trackDisabled = !trackDisabled));

        //while tracking is allowed and the robot is preparing to shoot or shooting, the driver
        //keeps translation and the robot turns itself to face the target
        new Trigger(this::isTracking).whileTrue(
                new TurnToTargetCmd(subsystems.drive, subsystems.vision,
                        () -> -driver.leftY(),
                        () -> -driver.leftX()));
        new Trigger(this::isShootTracking).whileTrue(
                new TurnToCmd(subsystems.drive,
                        subsystems.drive::getTargetHeadingRadians
                )
        );
    }

    public boolean isShootTracking(){
        return !trackDisabled && teleOpCycle.isIn(Cycle.SHOOT);
    }
    /**
     * True when the robot should be turning itself to the target: gamepad 1 has tracking toggled on,
     * gamepad 2 has not toggled the kill switch, and the state machine is preparing to shoot or shooting.
     */
    public boolean isTracking(){
        return trackEnabled
                && !trackDisabled
                && (teleOpCycle.isIn(Cycle.PREPARE));
    }

    public void configureOperator(){

    }

    private Trigger when(TeleOpContainer.Cycle c, Trigger button){
        return button.and(teleOpCycle.in(c));
    }

    public Command getStateMachine(){
        return teleOpCycle;
    }
}