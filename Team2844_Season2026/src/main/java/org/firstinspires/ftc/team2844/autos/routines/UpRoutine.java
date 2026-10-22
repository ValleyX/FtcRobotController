package org.firstinspires.ftc.team2844.autos.routines;

import com.pedropathing.api.PoseFactory;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.command.Commands;
import com.vcs.valleylib.ftc.pedro.PedroAutoDsl;

import org.firstinspires.ftc.team2844.autos.AutoBase;
import org.firstinspires.ftc.team2844.autos.geometries.UpGeometry;
import org.firstinspires.ftc.team2844.commands.CommandFactory;
import org.firstinspires.ftc.team2844.commands.shoot.ShootCmd;

public abstract class UpRoutine extends AutoBase {
    //-Yellow = just moving
    //!Red = shooting
    //?Blue = intaking
    @Override
    protected Command routine() {
        PoseFactory f = field();
        return PedroAutoDsl.auto(subsystems.drive, b -> {
            //!Shoot preloaded
            b.command(new ShootCmd(subsystems.intake, subsystems.nectar, subsystems.pollen, subsystems.transfer));

            //?Intake from the garden
            b.command(subsystems.drive.follow(UpGeometry.toGarden(f))
                    .deadlineWith(CommandFactory.intakeAndTransfer(subsystems.intake, subsystems.transfer)));

            //- Go back and wait for alliance to tip again
            b.follow(UpGeometry.readyShoot1(f));

            //!Wait for alliance to tip, then shoot
            b.command(()-> {
                subsystems.nectar.shoot();
                subsystems.pollen.shoot();
            });
            b.command(Commands.waitUntil(subsystems.vision::valid));
            b.command(new ShootCmd(subsystems.intake, subsystems.nectar, subsystems.pollen, subsystems.transfer));

            //?Intake from the flower on the alliance side wall
            b.parallel(
                    subsystems.drive.follow(UpGeometry.pickupYFlower(f))
                            .deadlineWith(CommandFactory.intakeAndTransfer(subsystems.intake, subsystems.transfer)),
                    Commands.waitUntil(() -> subsystems.drive.getFollower().completion() >= 0.6)
                            .andThen(() -> subsystems.intake.flowerRampDown())
                    );
            //- get back in position
            b.parallel(
                    subsystems.drive.follow(UpGeometry.readyShoot2(f)),
                    Commands.waitUntil(() -> subsystems.drive.getFollower().completion() >= 0.2)
                            .andThen(() -> {
                                subsystems.intake.flowerRampUp();
                            })
            );

            //!Shoot again and hopefully tip
            b.command(new ShootCmd(subsystems.intake, subsystems.nectar, subsystems.pollen, subsystems.transfer));
            //TODO add blob detection and state machine to keep shooting
        });
    }
}
