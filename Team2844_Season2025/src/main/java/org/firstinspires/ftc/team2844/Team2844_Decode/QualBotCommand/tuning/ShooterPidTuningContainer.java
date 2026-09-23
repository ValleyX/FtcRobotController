package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.tuning;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.vcs.valleylib.core.command.Commands;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.QualBotRobot;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.RobotConstants;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.teleop.TeleOpContainer;

/**
 * The normal teleop, with the second gamepad turned over to tuning the shooter's
 * velocity PIDF live.
 *
 * <table>
 *   <tr><td>dpad left / right</td><td>P by 0.1</td></tr>
 *   <tr><td>dpad down / up</td><td>I by 1</td></tr>
 *   <tr><td>a / y</td><td>D by 0.1</td></tr>
 *   <tr><td>b / x</td><td>F by 0.1</td></tr>
 * </table>
 */
public class ShooterPidTuningContainer extends TeleOpContainer {

    private static final double P_STEP = 0.1;
    private static final double I_STEP = 1.0;
    private static final double D_STEP = 0.1;
    private static final double F_STEP = 0.1;

    private double p = RobotConstants.SHOOTER_P;
    private double i = RobotConstants.SHOOTER_I;
    private double d = RobotConstants.SHOOTER_D;
    private double f = RobotConstants.SHOOTER_F;

    public ShooterPidTuningContainer(QualBotRobot robot, Gamepad gamepad1, Gamepad gamepad2) {
        super(robot, gamepad1, gamepad2);
    }

    @Override
    protected void configureOperator() {
        operator.dpadLeft().onTrue(adjust(-P_STEP, 0, 0, 0));
        operator.dpadRight().onTrue(adjust(P_STEP, 0, 0, 0));

        operator.dpadDown().onTrue(adjust(0, -I_STEP, 0, 0));
        operator.dpadUp().onTrue(adjust(0, I_STEP, 0, 0));

        operator.a().onTrue(adjust(0, 0, -D_STEP, 0));
        operator.y().onTrue(adjust(0, 0, D_STEP, 0));

        operator.b().onTrue(adjust(0, 0, 0, -F_STEP));
        operator.x().onTrue(adjust(0, 0, 0, F_STEP));
    }

    private com.vcs.valleylib.core.command.Command adjust(double dp, double di, double dd, double df) {
        return Commands.runOnce(() -> {
            p += dp;
            i += di;
            d += dd;
            f += df;
            robot.shooter.setVelocityPIDF(p, i, d, f);
        });
    }

    public double getP() {
        return p;
    }

    public double getI() {
        return i;
    }

    public double getD() {
        return d;
    }

    public double getF() {
        return f;
    }
}
