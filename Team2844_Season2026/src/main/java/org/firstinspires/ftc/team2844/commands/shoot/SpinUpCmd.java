package org.firstinspires.ftc.team2844.commands.shoot;

import com.vcs.valleylib.core.command.Command;

import org.firstinspires.ftc.team2844.subsystems.ShooterSubsystem;

public class SpinUpCmd implements Command {
    private final ShooterSubsystem nectar, pollen;
    public SpinUpCmd(ShooterSubsystem nectar, ShooterSubsystem pollen){
        this.nectar = nectar;
        this.pollen = pollen;
    }

    @Override
    public void execute() {
        nectar.shoot();
        pollen.shoot();
    }

    @Override
    public boolean isFinished() {
        return nectar.atVelocity() && pollen.atVelocity();
    }
}
