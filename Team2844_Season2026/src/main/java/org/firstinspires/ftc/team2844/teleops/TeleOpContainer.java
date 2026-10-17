package org.firstinspires.ftc.team2844.teleops;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.command.Commands;
import com.vcs.valleylib.core.command.InstantCommand;
import com.vcs.valleylib.ftc.RobotContainer;
import com.vcs.valleylib.ftc.input.CommandGamepad;
import com.vcs.valleylib.ftc.input.Trigger;

import org.firstinspires.ftc.team2844.commands.shoot.IdleShootCmd;
import org.firstinspires.ftc.team2844.helpers.Constants;
import org.firstinspires.ftc.team2844.helpers.Subsystems;

public class TeleOpContainer extends RobotContainer {
    Subsystems subsystems;
    CommandGamepad driver;
    CommandGamepad operator;
    Trigger shootButton;

    public TeleOpContainer(Subsystems subsystems, Gamepad gamepad1, Gamepad gamepad2) {
        this.subsystems = subsystems;
        this.driver = CommandGamepad.forLogitechF310(gamepad1);
        this.operator = CommandGamepad.forLogitechF310(gamepad2);
        this.shootButton = driver.rightBumper();

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
        subsystems.drive.setDefaultCommand(subsystems.drive.run(
                () -> subsystems.drive.driveFieldCentric(
                        -driver.leftY(),
                        -driver.leftX(),
                        -driver.rightX()
                )
        ));

        subsystems.nectar.setDefaultCommand(new IdleShootCmd(subsystems.nectar));
        subsystems.pollen.setDefaultCommand(new IdleShootCmd(subsystems.pollen));

        Trigger audienceSide = new Trigger(
                () -> subsystems.drive.getPose().y() > 72);

        audienceSide.onTrue(
                new InstantCommand(() ->subsystems.vision.setPipeline(Constants.alliance + Constants.AUDIENCE_SIDE))
                .andThen(() ->Constants.changeTarget(Constants.alliance + Constants.AUDIENCE_SIDE)));
        audienceSide.onFalse(new InstantCommand(() ->subsystems.vision.setPipeline(Constants.alliance + Constants.BACK_SIDE))
                .andThen(() ->Constants.changeTarget(Constants.alliance + Constants.BACK_SIDE)));
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
