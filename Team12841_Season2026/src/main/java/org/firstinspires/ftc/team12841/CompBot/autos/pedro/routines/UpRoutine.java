package org.firstinspires.ftc.team12841.CompBot.autos.pedro.routines;

import com.pedropathing.api.PoseFactory;
import com.vcs.valleylib.core.auto.AutoDsl;
import com.vcs.valleylib.core.command.Command;

import org.firstinspires.ftc.team12841.CompBot.autos.pedro.AutoBase;
import org.firstinspires.ftc.team12841.CompBot.autos.pedro.FollowCmd;
import org.firstinspires.ftc.team12841.CompBot.autos.pedro.geometry.UpGeometry;

public abstract class UpRoutine extends AutoBase {
    @Override
    protected Command routine() {
        PoseFactory f = field();
        return AutoDsl.auto(b -> {
            b.run(()->robot.setDrumSpeedPower(0.5));
            b.run(()->robot.setFeederMotorPower(1.0));
            b.waitFor(3);
            b.run(()->robot.setDrumSpeedPower(0.2));
            b.run(()->robot.setFeederMotorPower(0.0));
            b.command(new FollowCmd(robot, UpGeometry.getPath(f)));
        });
    }
}
