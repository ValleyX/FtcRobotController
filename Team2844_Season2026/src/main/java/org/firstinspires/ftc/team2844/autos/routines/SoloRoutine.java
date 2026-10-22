package org.firstinspires.ftc.team2844.autos.routines;

import com.pedropathing.api.PoseFactory;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.command.Commands;
import com.vcs.valleylib.ftc.pedro.PedroAutoDsl;

import org.firstinspires.ftc.team2844.autos.AutoBase;
import org.firstinspires.ftc.team2844.autos.geometries.SoloGeometry;
import org.firstinspires.ftc.team2844.commands.intake.IntakeCmd;
import org.firstinspires.ftc.team2844.commands.shoot.ShootCmd;
import org.firstinspires.ftc.team2844.helpers.Constants;

public abstract class SoloRoutine extends AutoBase {
    @Override
    public Command routine() {
        PoseFactory f = field();
        return PedroAutoDsl.auto(subsystems.drive, b -> {
            b.command(new ShootCmd(subsystems.intake, subsystems.nectar, subsystems.pollen, subsystems.transfer));
            b.command(new IntakeCmd(subsystems.intake));
            b.follow(SoloGeometry.toGarden(f));
            b.follow(SoloGeometry.underpassFromGarden(f));
            b.action(() -> Constants.changeSide(Constants.AUDIENCE_SIDE));
            b.command(new ShootCmd(subsystems.intake, subsystems.nectar, subsystems.pollen, subsystems.transfer));
            b.command(new IntakeCmd(subsystems.intake));
            b.parallel(
                    subsystems.drive.follow(SoloGeometry.pickupXFlower(f)),
                    Commands.waitUntil(() -> subsystems.drive.getFollower().completion() >= 0.6)
                            .andThen(() -> subsystems.intake.flowerRampDown())
            );
            b.parallel(
                    subsystems.drive.follow(SoloGeometry.shootAgain(f)),
                    Commands.waitUntil(() -> subsystems.drive.getFollower().completion() >= 0.2)
                            .andThen(() -> subsystems.intake.flowerRampUp())
            );
            //TODO add blob detection and state machine to keep shooting
        });
    }
}
