package org.firstinspires.ftc.team2844.teleops;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.ftc.RobotContainer;
import com.vcs.valleylib.ftc.input.CommandGamepad;
import com.vcs.valleylib.ftc.input.Trigger;

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

    }

    public void configureDriving(){

    }

    public void configureIntake(){

    }

    public void configureShooting(){

    }

    public void configureOperator(){

    }
}
