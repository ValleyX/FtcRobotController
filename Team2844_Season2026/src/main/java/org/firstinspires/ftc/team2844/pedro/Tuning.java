package org.firstinspires.ftc.team2844.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.team2844.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.team2844.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.team2844.pedro.procedures.OTOSTuner;
import org.firstinspires.ftc.team2844.pedro.procedures.OctoQuadTuner;
import org.firstinspires.ftc.team2844.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.team2844.pedro.procedures.Tests;
import org.firstinspires.ftc.team2844.pedro.procedures.ThreeWheelIMUTuner;
import org.firstinspires.ftc.team2844.pedro.procedures.ThreeWheelTuner;
import org.firstinspires.ftc.team2844.pedro.procedures.TwoWheelTuner;

/**
 * Registers Pedro's tuning procedures with the autotuner.
 *
 * <p>Nothing calls these methods directly. Pedro's {@code TunerScanner} walks the
 * APK at startup looking for static, no-argument methods annotated
 * {@link Tuner} that return a {@link Procedure}, and registers whatever it finds.
 * So adding a tuner is just adding a method here.
 *
 * <p>To actually run them: start the <b>Pedro Tuning</b> OpMode on the robot, then
 * open the web UI it serves from the Control Hub in a browser. Each procedure
 * finishes by printing a Java config block to paste into {@link PedroConstants}.
 *
 * <h2>Order to run them in</h2>
 * <ol>
 *   <li>{@link #mecanumTuner()} — wheel names and directions.</li>
 *   <li>{@link #pinpointTuner()} — this robot's localizer.</li>
 *   <li>{@link #foresightTuner()} — needs the first two correct, because it
 *       drives the robot and measures how it actually moves.</li>
 *   <li>{@link #tests()} — hold, line, curve, interpolation, localization and raw
 *       driving checks.</li>
 * </ol>
 *
 * <p>The other localizer tuners are registered too, so they are there if the
 * odometry hardware ever changes. They cost nothing when unused.
 */
public class Tuning {

    /* ===================== Drivetrain ===================== */

    @Tuner(name = "Mecanum Tuner")
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    /* ===================== Localizers ===================== */

    /** This robot: goBILDA Pinpoint odometry computer. */
    @Tuner(name = "Pinpoint Tuner")
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner(name = "Two Wheel Tuner")
    public static Procedure twoWheelTuner() {
        return new TwoWheelTuner();
    }

    @Tuner(name = "Three Wheel Tuner")
    public static Procedure threeWheelTuner() {
        return new ThreeWheelTuner();
    }

    @Tuner(name = "Three Wheel + IMU Tuner")
    public static Procedure threeWheelImuTuner() {
        return new ThreeWheelIMUTuner();
    }

    @Tuner(name = "OTOS Tuner")
    public static Procedure otosTuner() {
        return new OTOSTuner();
    }

    @Tuner(name = "OctoQuad Tuner")
    public static Procedure octoQuadTuner() {
        return new OctoQuadTuner();
    }

    /* ===================== Following and verification ===================== */

    /**
     * Note the argument order: ForesightTuner takes (localizer, drivetrain).
     * Using method references off {@link PedroConstants} rather than lambdas makes a
     * transposition a compile error instead of a very confusing robot.
     */
    @Tuner(name = "Foresight Tuner")
    public static Procedure foresightTuner() {
        return new ForesightTuner(PedroConstants::createLocalizer, PedroConstants::createDrivetrain);
    }

    /** And Tests takes (drivetrain, localizer, algorithm) -- a different order. */
    @Tuner(name = "Tests")
    public static Procedure tests() {
        return new Tests(
                PedroConstants::createDrivetrain,
                PedroConstants::createLocalizer,
                PedroConstants::createAlgorithm);
    }
}
