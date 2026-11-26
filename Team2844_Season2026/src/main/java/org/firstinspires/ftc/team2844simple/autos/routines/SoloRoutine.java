package org.firstinspires.ftc.team2844simple.autos.routines;

import com.pedropathing.api.PoseFactory;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.command.Commands;
import com.vcs.valleylib.ftc.pedro.PedroAutoDsl;

import org.firstinspires.ftc.team2844simple.autos.AutoBase;
import org.firstinspires.ftc.team2844simple.autos.geometries.SoloGeometry;
import org.firstinspires.ftc.team2844simple.commands.CommandFactory;
import org.firstinspires.ftc.team2844simple.commands.shoot.ShootCmd;
import org.firstinspires.ftc.team2844simple.helpers.Constants;
import org.firstinspires.ftc.team2844global.GlobalConstants;
import org.firstinspires.ftc.team2844global.MatchInfo;

public abstract class SoloRoutine extends AutoBase {
    //-Yellow = just moving
    //!Red = shooting
    //?Blue = intaking
    @Override
    protected Command routine() {
        PoseFactory f = field();
        return PedroAutoDsl.auto(subsystems.drive, b -> {
            //!Shoot preloaded immediately
            b.command(new ShootCmd(subsystems.intake, subsystems.pollen, subsystems.transfer, 4000));

            //?Grab from the garden
            b.command(subsystems.drive.follow(SoloGeometry.toGarden(f))
                    .deadlineWith(CommandFactory.intakeAndTransfer(subsystems.intake, subsystems.transfer))
                    .withTimeout(6));

            //-Go through the middle to the other side (to avoid hitting parking teammate)
            b.follow(SoloGeometry.underpassFromGarden(f));
            b.action(() ->
                MatchInfo.changeSide((MatchInfo.side == GlobalConstants.AUDIENCE_SIDE)? GlobalConstants.BACK_SIDE: GlobalConstants.AUDIENCE_SIDE));

            //!We assume that the hive has tipped from our first shot
            b.command(new ShootCmd(subsystems.intake, subsystems.pollen, subsystems.transfer, 4000));
        });
    }
}
