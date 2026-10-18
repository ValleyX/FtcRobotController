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
import org.firstinspires.ftc.team2844.commands.shoot.IdleShootCmd;
import org.firstinspires.ftc.team2844.helpers.Constants;
import org.firstinspires.ftc.team2844.helpers.Subsystems;

public class TeleOpContainer extends RobotContainer {
    public enum Cycle {INTAKE, SHOOT};

    Subsystems subsystems;
    CommandGamepad driver;
    CommandGamepad operator;
    Trigger shootButton;
    StateMachine<Cycle> teleOpCycle;
    CommandFactory factory;

    public TeleOpContainer(Subsystems subsystems, Gamepad gamepad1, Gamepad gamepad2) {
        this.subsystems = subsystems;
        factory = new CommandFactory(subsystems);
        this.driver = CommandGamepad.forLogitechF310(gamepad1);
        this.operator = CommandGamepad.forLogitechF310(gamepad2);
        this.shootButton = driver.rightBumper();

        teleOpCycle = new StateMachine<>(Cycle.INTAKE)
                .state(Cycle.INTAKE, factory.IntakeState())
                .state(Cycle.SHOOT, factory.ShootState())
                .transitionOnFinish(Cycle.INTAKE, Cycle.SHOOT)
                .transitionOnFinish(Cycle.SHOOT, Cycle.INTAKE);
        //TODO add real commands for states, and transition when full and empty.
        //TODO add shooting state that turns off intake and transfer until velocity is in range and opens gates
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
                () -> subsystems.drive.getPose().y() > 72);

        //when the robot goes to the audience side, set the pipeline and target
        audienceSide.onTrue(
                new InstantCommand(() ->subsystems.vision.setPipeline(Constants.alliance + Constants.AUDIENCE_SIDE))
                .andThen(() ->Constants.changeTarget(Constants.alliance + Constants.AUDIENCE_SIDE)));
        //when the robot goes to the back side, set the pipeline and target
        audienceSide.onFalse(new InstantCommand(() ->subsystems.vision.setPipeline(Constants.alliance + Constants.BACK_SIDE))
                .andThen(() ->Constants.changeTarget(Constants.alliance + Constants.BACK_SIDE)));


        //StateMachine
        subsystems.nectar.setDefaultCommand(teleOpCycle);
        subsystems.pollen.setDefaultCommand(teleOpCycle);
        subsystems.intake.setDefaultCommand(teleOpCycle);
        subsystems.transfer.setDefaultCommand(teleOpCycle);
        subsystems.turret.setDefaultCommand(teleOpCycle);
    }

    public void configureDriving(){
        driver.guide().onTrue(
                Commands.runOnce(subsystems.drive::resetHeading)
        );
    }

    public void configureIntake(){

    }

    public void configureShooting(){

    }

    public void configureOperator(){

    }
}
