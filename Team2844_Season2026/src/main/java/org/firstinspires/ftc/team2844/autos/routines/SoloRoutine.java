package org.firstinspires.ftc.team2844.autos.routines;

import com.pedropathing.api.PoseFactory;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.command.Commands;
import com.vcs.valleylib.ftc.pedro.PedroAutoDsl;

import org.firstinspires.ftc.team2844.autos.AutoBase;
import org.firstinspires.ftc.team2844.autos.geometries.SoloGeometry;
import org.firstinspires.ftc.team2844.commands.CommandFactory;
import org.firstinspires.ftc.team2844.commands.intake.IntakeCmd;
import org.firstinspires.ftc.team2844.commands.shoot.ShootCmd;
import org.firstinspires.ftc.team2844.helpers.Constants;

public abstract class SoloRoutine extends AutoBase {
    //-Yellow = just moving
    //!Red = shooting
    //?Blue = intaking
    @Override
    protected Command routine() {
        PoseFactory f = field();
        return PedroAutoDsl.auto(subsystems.drive, b -> {
            //!Shoot preloaded immediately
            b.command(new ShootCmd(subsystems.intake, subsystems.nectar, subsystems.pollen, subsystems.transfer));

            //?Grab from the garden
            b.command(subsystems.drive.follow(SoloGeometry.toGarden(f))
                    .deadlineWith(CommandFactory.intakeAndTransfer(subsystems.intake, subsystems.transfer)));

            //-Go through the middle to the other side (to avoid hitting parking teammate)
            b.follow(SoloGeometry.underpassFromGarden(f));
            b.action(() -> Constants.changeSide(Constants.AUDIENCE_SIDE));

            //!We assume that the hive has tipped from our first shot
            b.command(new ShootCmd(subsystems.intake, subsystems.nectar, subsystems.pollen, subsystems.transfer));

            //?Intake from the flowers (might have overcomplicated it)
            b.parallel(
                    subsystems.drive.follow(SoloGeometry.pickupXFlower(f))
                            .deadlineWith(CommandFactory.intakeAndTransfer(subsystems.intake, subsystems.transfer)),
                    Commands.waitUntil(() -> subsystems.drive.getFollower().completion() >= 0.2)
                            .andThen(() -> subsystems.intake.flowerRampDown())
            );
            b.parallel(
                    subsystems.drive.follow(SoloGeometry.shootAgain(f)),
                    Commands.waitUntil(() -> subsystems.drive.getFollower().completion() >= 0.8)
                            .andThen(() -> subsystems.intake.flowerRampUp())
            );

            //!Shoot the pollen collected from the flowers
            b.command(new ShootCmd(subsystems.intake, subsystems.nectar, subsystems.pollen, subsystems.transfer));
            //TODO add blob detection and state machine to keep shooting
        });
    }
}
