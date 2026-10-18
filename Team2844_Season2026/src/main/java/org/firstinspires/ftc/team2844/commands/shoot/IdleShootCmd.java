package org.firstinspires.ftc.team2844.commands.shoot;

import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.subsystem.Subsystem;

import org.firstinspires.ftc.team2844.helpers.Constants;
import org.firstinspires.ftc.team2844.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.subsystems.TransferSubsystem;

import java.util.Collections;
import java.util.Set;

public class IdleShootCmd implements Command {
    ShooterSubsystem shoot;
    public IdleShootCmd(ShooterSubsystem shoot){
        this.shoot = shoot;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return  Collections.singleton(shoot);
    }

    @Override
    public void execute() {
        shoot.shoot(Constants.SHOOTER_IDLE, Constants.HOOD_DEFAULT);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
