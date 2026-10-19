package org.firstinspires.ftc.team2844.teleops;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.command.Commands;
import com.vcs.valleylib.core.command.InstantCommand;
import com.vcs.valleylib.core.fsm.StateMachine;
import com.vcs.valleylib.ftc.RobotContainer;
import com.vcs.valleylib.ftc.input.CommandGamepad;
import com.vcs.valleylib.ftc.input.Trigger;

import org.firstinspires.ftc.team2844.commands.CommandFactory;
import org.firstinspires.ftc.team2844.commands.turret.DefaultTurretCmd;
import org.firstinspires.ftc.team2844.helpers.Constants;
import org.firstinspires.ftc.team2844.helpers.Subsystems;

public class TeleOpContainer extends RobotContainer {
    public enum Cycle {IDLE, INTAKE, PREPARE, SHOOT};

    Subsystems subsystems;

    /* ---------- Controller and Buttons ---------- */
    CommandGamepad driver;
    CommandGamepad operator;
    final Trigger shootButton;
    final Trigger intakeButton;
    final Trigger outtakeButton;
    final Trigger resetIMUButton;
    final Trigger aimButton;
    final Trigger turretLeftButton;
    final Trigger turretRightButton;
    final Trigger turretCenterButton;
    final Trigger switchManual;
    final Trigger relativeSwitch;




    StateMachine<Cycle> teleOpCycle;
    CommandFactory factory;
    DefaultTurretCmd defaultTurretCmd;

    public TeleOpContainer(Subsystems subsystems, Gamepad gamepad1, Gamepad gamepad2) {
        this.subsystems = subsystems;
        factory = new CommandFactory(subsystems);
        defaultTurretCmd = new DefaultTurretCmd(subsystems.turret, subsystems.vision);

        this.driver = CommandGamepad.forLogitechF310(gamepad1);
        this.operator = CommandGamepad.forLogitechF310(gamepad2);

        /* ---------- CONTROLLER BINDINGS ---------- */
        // ----- DRIVER ----- //
        this.shootButton = driver.rightBumper();
        this.intakeButton = driver.rightTriggerButton(0.05); //runs the intake
        this.outtakeButton = driver.leftTriggerButton(0.05); //runs the intake backwards
        this.resetIMUButton = driver.guide();

        // ----- OPERATOR ----- //
        this.aimButton = operator.rightBumper();//Automatically aims at expected target position, then apriltag target position
        this.turretLeftButton = operator.x();   //turns the turret to the Left of robot
        this.turretCenterButton = operator.a(); //turns the turret to face center of robot
        this.turretRightButton = operator.b();  //turns the turret to face Right of robot
        this.switchManual = operator.y();      //Lets the state Machine take back over
        this.relativeSwitch = operator.leftBumper();




        teleOpCycle = new StateMachine<>(Cycle.IDLE)
                // IDLE: close the gates then turn on the transfer
                .onEnter(Cycle.IDLE, subsystems.transfer::closeGates)
                .state(Cycle.IDLE, () -> {
                    subsystems.transfer.transfer();
                    subsystems.nectar.idleShoot();
                    subsystems.pollen.idleShoot();
                })
                //INTAKE: turn on the intake and when finished, turn it off
                .state(Cycle.INTAKE, subsystems.intake::intake)
                .onExit(Cycle.INTAKE, subsystems.intake::stopIntake)

                //PREPARE: turn off transfer, open gates, aim the turret to the target, spinup both shooters
                .onEnter(Cycle.PREPARE, () -> {
                    subsystems.transfer.stop();
                    subsystems.transfer.openGates();
                })
                .state(Cycle.PREPARE, () -> {
                    subsystems.nectar.shoot();
                    subsystems.pollen.shoot();
                    defaultTurretCmd.setTrack(true);
                })
                //SHOOT: transfers balls into shooter when at velocity(due to transition requirements)
                .state(Cycle.SHOOT, () -> {
                    subsystems.nectar.shoot();
                    subsystems.pollen.shoot();
                    defaultTurretCmd.setTrack(true);
                    subsystems.transfer.transfer();
                })
                .onExit(Cycle.SHOOT, () -> defaultTurretCmd.setTrack(false))

                //acts like an intake button with proper procedure
                .transition(Cycle.IDLE, Cycle.INTAKE, intakeButton)
                .transition(Cycle.INTAKE, Cycle.IDLE, intakeButton.negate())
                //Acts like a shoot button with proper procedure
                .transition(Cycle.PREPARE, Cycle.SHOOT,
                        shootButton
                                .and(subsystems.nectar::atVelocity)
                                .and(subsystems.pollen::atVelocity)
                                .and(subsystems.turret::aimed))
                .transition(Cycle.SHOOT, Cycle.PREPARE, shootButton.negate())

                //Switches from idling or intaking to preparing for the shooter when it has 4 balls
                .transition(Cycle.INTAKE, Cycle.PREPARE, subsystems.intake::full)
                .transition(Cycle.IDLE, Cycle.PREPARE, subsystems.intake::full)
                //Switches from shooting or preparing to idling when it is empty
                .transition(Cycle.PREPARE, Cycle.IDLE, subsystems.intake::empty)
                .transition(Cycle.SHOOT, Cycle.IDLE, subsystems.intake::empty)

                //Fast track to shooting/intaking
                .transition(Cycle.IDLE, Cycle.PREPARE, shootButton.and(() -> !subsystems.intake.empty()))
                .transition(Cycle.PREPARE, Cycle.IDLE, intakeButton.and(() -> !subsystems.intake.full()));
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
        //drive in fieldCentric
        subsystems.drive.setDefaultCommand(subsystems.drive.run(
                () -> subsystems.drive.driveFieldCentric(
                        -driver.leftY(),
                        -driver.leftX(),
                        -driver.rightX()
                )
        ));

        //Always checks the position of the robot and changes the pipeline and target based on that
        Trigger audienceSide = new Trigger(
                () -> subsystems.drive.getPose().y() < 72);

        //when the robot goes to the audience side, set the pipeline and target
        audienceSide.onTrue(
                new InstantCommand(() ->subsystems.vision.setPipeline(Constants.alliance + Constants.AUDIENCE_SIDE))
                .andThen(() ->Constants.changeTarget(Constants.alliance + Constants.AUDIENCE_SIDE)));
        //when the robot goes to the back side, set the pipeline and target
        audienceSide.onFalse(new InstantCommand(() ->subsystems.vision.setPipeline(Constants.alliance + Constants.BACK_SIDE))
                .andThen(() ->Constants.changeTarget(Constants.alliance + Constants.BACK_SIDE)));

        subsystems.turret.setDefaultCommand(defaultTurretCmd);
    }

    public void configureDriving(){
        resetIMUButton.onTrue(
                Commands.runOnce(subsystems.drive::resetHeading)
        );
    }

    public void configureIntake(){

    }

    public void configureShooting(){
        turretLeftButton.onTrue(new InstantCommand(() -> {
            defaultTurretCmd.setDegrees(90.0);
            defaultTurretCmd.setManual(true);
        }));
        turretCenterButton.onTrue(new InstantCommand(() -> {
            defaultTurretCmd.setDegrees(0.0);
            defaultTurretCmd.setManual(true);
        }));
        turretRightButton.onTrue(new InstantCommand(() -> {
            defaultTurretCmd.setDegrees(-90.0);
            defaultTurretCmd.setManual(true);
        }));
        switchManual.onTrue(new InstantCommand(() -> defaultTurretCmd.setManual(!defaultTurretCmd.getManual())));
        relativeSwitch.onTrue(new InstantCommand(() -> defaultTurretCmd.setRelative(!defaultTurretCmd.getRelative())));

        aimButton.onTrue(new InstantCommand(() -> {
            defaultTurretCmd.setManual(false);
            defaultTurretCmd.setTrack(true);
        }));
    }

    public void configureOperator(){

    }
}