package org.firstinspires.ftc.teamcode.team2844simple;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.team2844simple.pedro.PedroConstants;
import org.firstinspires.ftc.team2844simple.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.team2844simple.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.team2844simple.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.team2844simple.pedro.procedures.Tests;

/**
 * Registers Pedro's tuning procedures with the autotuner, for the team2844simple robot.
 *
 * <p>Copy of {@code org.firstinspires.ftc.teamcode.pedro.Tuning}, pointed at
 * {@code team2844simple.pedro} instead of {@code team2844.pedro}. The old class is still
 * registered too, so the autotuner lists both sets; every tuner here is prefixed
 * "Simple" so you can tell them apart.
 *
 * <h2>Why this class is in org.firstinspires.ftc.teamcode and not team2844simple</h2>
 * It has to be, or nothing here runs. Pedro's {@code TunerScanner} targets
 * Sinister's {@code NarrowSearch}, which is built as:
 *
 * <pre>
 *     exclude("org.firstinspires");
 *     include("org.firstinspires.ftc.teamcode");
 *     exclude("com.qualcomm");
 * </pre>
 *
 * Those are prefix rules and the most specific one wins, so
 * {@code org.firstinspires.ftc.teamcode} is the <b>only</b> package under
 * {@code org.firstinspires} that gets scanned. A tuner class in
 * {@code org.firstinspires.ftc.team2844simple} is silently skipped.
 *
 * <p>Only this class needs to be scannable. The {@code Procedure} classes in
 * {@code team2844simple.pedro.procedures} and {@link PedroConstants} are plain objects
 * that these methods construct, so they can stay where they are.
 *
 * <p>Nothing calls these methods directly. Pedro's {@code TunerScanner} walks the
 * APK at startup looking for static, no-argument methods annotated
 * {@link Tuner} that return a {@link Procedure}, and registers whatever it finds.
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
 */
public class SimpleTuning {

    /* ===================== Drivetrain ===================== */

    @Tuner(name = "Simple Mecanum Tuner")
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    /* ===================== Localizer ===================== */

    /** This robot: goBILDA Pinpoint odometry computer. */
    @Tuner(name = "Simple Pinpoint Tuner")
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    /* ===================== Following and verification ===================== */

    /**
     * Note the argument order: ForesightTuner takes (localizer, drivetrain).
     * Using method references off {@link PedroConstants} rather than lambdas makes a
     * transposition a compile error instead of a very confusing robot.
     */
    @Tuner(name = "Simple Foresight Tuner")
    public static Procedure foresightTuner() {
        return new ForesightTuner(PedroConstants::createLocalizer, PedroConstants::createDrivetrain);
    }

    /** And Tests takes (drivetrain, localizer, algorithm) -- a different order. */
    @Tuner(name = "Simple Tests")
    public static Procedure tests() {
        return new Tests(
                PedroConstants::createDrivetrain,
                PedroConstants::createLocalizer,
                PedroConstants::createAlgorithm);
    }
}
