package org.firstinspires.ftc.team2844simple.autos.routines;

import com.pedropathing.api.PoseFactory;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.command.Commands;
import com.vcs.valleylib.ftc.pedro.PedroAutoDsl;

import org.firstinspires.ftc.team2844simple.autos.AutoBase;
import org.firstinspires.ftc.team2844simple.autos.geometries.DownGeometry;
import org.firstinspires.ftc.team2844simple.commands.CommandFactory;
import org.firstinspires.ftc.team2844simple.commands.shoot.ShootCmd;

public abstract class DownRoutine extends AutoBase {
    //-Yellow = just moving
    //!Red = shooting
    //?Blue = intaking
    @Override
    protected Command routine() {
        PoseFactory f = field();
        return PedroAutoDsl.auto(subsystems.drive, b -> {
            //! just spinUp till you can see the aprilTag(goal is up) then shoot
            b.doInstant(()-> {
                subsystems.pollen.shoot();
            });
            b.waitFor(15.0);
            b.command(new ShootCmd(subsystems.intake, subsystems.pollen, subsystems.transfer, 8000));
        });
    }
}
