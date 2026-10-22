package org.firstinspires.ftc.team2844.autos.routines;

import com.pedropathing.api.PoseFactory;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.command.Commands;
import com.vcs.valleylib.ftc.pedro.PedroAutoDsl;

import org.firstinspires.ftc.team2844.autos.AutoBase;
import org.firstinspires.ftc.team2844.autos.geometries.DownGeometry;
import org.firstinspires.ftc.team2844.commands.shoot.ShootCmd;

public abstract class DownRoutine extends AutoBase {
    @Override
    protected Command routine() {
        PoseFactory f = field();
        return PedroAutoDsl.auto(subsystems.drive, b -> {
            b.follow(DownGeometry.readyShoot(f));
            b.command(Commands.waitUntil(subsystems.vision::valid));
            b.command(new ShootCmd(subsystems.intake, subsystems.nectar, subsystems.pollen, subsystems.transfer));
            b.command(DownGeometry.turnToFlower(subsystems.drive));
            b.doInstant(subsystems.intake::flowerRampDown);
            b.follow(DownGeometry.pickupXFlower(f));
            b.follow(DownGeometry.backToShoot(f));
            b.doInstant(subsystems.intake::flowerRampUp);
            b.command(Commands.waitUntil(subsystems.vision::valid));
            b.command(new ShootCmd(subsystems.intake, subsystems.nectar, subsystems.pollen, subsystems.transfer));
            //TODO add statemachine to pickup blobs
        });
    }
}
