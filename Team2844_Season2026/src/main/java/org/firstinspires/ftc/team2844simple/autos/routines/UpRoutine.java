package org.firstinspires.ftc.team2844simple.autos.routines;

import com.pedropathing.api.PoseFactory;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.command.Commands;
import com.vcs.valleylib.ftc.pedro.PedroAutoDsl;

import org.firstinspires.ftc.team2844simple.autos.AutoBase;
import org.firstinspires.ftc.team2844simple.autos.geometries.UpGeometry;
import org.firstinspires.ftc.team2844simple.commands.CommandFactory;
import org.firstinspires.ftc.team2844simple.commands.shoot.ShootCmd;

public abstract class UpRoutine extends AutoBase {
    //-Yellow = just moving
    //!Red = shooting
    //?Blue = intaking
    @Override
    protected Command routine() {
        PoseFactory f = field();
        return PedroAutoDsl.auto(subsystems.drive, b -> {
            //!Shoot preloaded
            b.command(new ShootCmd(subsystems.intake, subsystems.pollen, subsystems.transfer));

            //?Intake from the garden
            b.command(subsystems.drive.follow(UpGeometry.toGarden(f))
                    .deadlineWith(CommandFactory.intakeAndTransfer(subsystems.intake, subsystems.transfer)));

            //- Go back and wait for alliance to tip again
            b.follow(UpGeometry.readyShoot1(f));

            //!Spin up the shooters
            b.doInstant(()-> {
                subsystems.pollen.shoot();
            });
            //!Wait for alliance to tip, then shoot
            b.command(Commands.waitUntil(subsystems.vision::valid));
            b.command(new ShootCmd(subsystems.intake, subsystems.pollen, subsystems.transfer));

            //?Intake from the flower on the alliance side wall
            b.command(
                    subsystems.drive.follow(UpGeometry.pickupYFlower(f))
                            .deadlineWith(CommandFactory.intakeAndTransfer(subsystems.intake, subsystems.transfer))
                    );
            //- get back in position
            b.command(
                    subsystems.drive.follow(UpGeometry.readyShoot2(f))
            );

            //!Shoot again and hopefully tip
            b.command(new ShootCmd(subsystems.intake, subsystems.pollen, subsystems.transfer));
            //TODO add blob detection and state machine to keep shooting
        });
    }
}
