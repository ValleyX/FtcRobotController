package org.firstinspires.ftc.team2844.autos.routines;

import com.pedropathing.api.PoseFactory;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.command.Commands;
import com.vcs.valleylib.ftc.pedro.PedroAutoDsl;

import org.firstinspires.ftc.team2844.autos.AutoBase;
import org.firstinspires.ftc.team2844.autos.geometries.DownGeometry;
import org.firstinspires.ftc.team2844.commands.shoot.ShootCmd;
import org.firstinspires.ftc.team2844.commands.shoot.SpinUpCmd;

public abstract class DownRoutine extends AutoBase {
    //-Yellow = just moving
    //!Red = shooting
    //?Blue = intaking
    @Override
    protected Command routine() {
        PoseFactory f = field();
        return PedroAutoDsl.auto(subsystems.drive, b -> {
            //-line up in front of the flower with limelight facing the goal
            b.follow(DownGeometry.readyShoot(f));

            //! just spinUp till you can see the aprilTag(goal is up) then shoot
            b.command(new SpinUpCmd(subsystems.nectar, subsystems.pollen));
            b.command(Commands.waitUntil(subsystems.vision::valid));
            b.command(new ShootCmd(subsystems.intake, subsystems.nectar, subsystems.pollen, subsystems.transfer));

            //? Intake from flower
            b.command(DownGeometry.turnToFlower(subsystems.drive));
            b.doInstant(subsystems.intake::flowerRampDown);
            b.follow(DownGeometry.pickupXFlower(f));

            //! Shoot again
            b.follow(DownGeometry.backToShoot(f));
            b.doInstant(subsystems.intake::flowerRampUp);
            b.command(new ShootCmd(subsystems.intake, subsystems.nectar, subsystems.pollen, subsystems.transfer));

            //TODO add statemachine to pickup blobs
        });
    }
}
