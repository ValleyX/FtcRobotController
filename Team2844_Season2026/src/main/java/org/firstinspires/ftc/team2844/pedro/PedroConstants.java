package org.firstinspires.ftc.team2844.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team2844.helpers.Constants;

/**
 * Pedro Pathing 3 configuration: mecanum drivetrain with a goBILDA Pinpoint
 * odometry computer.
 *
 * <h2>How to fill this in</h2>
 * Don't hand-measure these. Run the <b>Pedro Tuning</b> OpMode on the robot, open
 * the web UI it serves, and work through the procedures — each one ends by
 * printing a finished Java config block that you paste over the matching method
 * below. Every field here is laid out in exactly the shape the tuners emit, so it
 * should be a straight copy-paste.
 *
 * <ol>
 *   <li><b>Mecanum Tuner</b> — spins each wheel in turn and asks which way it
 *       went. Emits {@link #drivetrainConfig()}.</li>
 *   <li><b>Pinpoint Tuner</b> — pod type, pod directions, and the two pod
 *       offsets. Emits {@link #localizerConfig()}.</li>
 *   <li><b>Foresight Tuner</b> — velocities, decelerations, braking model and the
 *       controller gains. Emits {@link #foresightConfig()}. Run this last: it
 *       drives the robot and needs the first two to be correct already.</li>
 *   <li><b>Tests</b> — hold, line, curve, interpolation, localization and raw
 *       driving checks. Run these before trusting an auto.</li>
 * </ol>
 *
 * <p>Until the tuners have been run, the values below are placeholders — the
 * drivetrain and Pinpoint entries are marked TODO and will not match this robot,
 * and the Foresight numbers are Pedro's stock figures. Do not run a path against
 * them.
 *
 * <h2>Coordinates</h2>
 * A Pedro 3 {@link com.pedropathing.math.Pose} is a bare (x, y, heading) with no
 * frame attached: the field frame is simply whatever pose the localizer is given
 * at startup. Pick one convention and keep to it. FTC-standard centre-origin
 * (+-72 inches, +x toward the red wall, +y to its left) is a reasonable default,
 * and {@link com.pedropathing.api.PoseFactory} will build poses in degrees and
 * mirror a whole alliance's worth for you.
 */
public class PedroConstants {

    /* ===================== Drivetrain ===================== */

    /**
     * TODO replace with the output of the <b>Mecanum Tuner</b>.
     *
     * <p>The motor names must match the robot configuration on the hub, and the
     * four directions are what the tuner determines by spinning each wheel.
     */
    public static MecanumConfig drivetrainConfig() {
        return new MecanumConfig(c -> {
            c.frontLeftName.set(Constants.CHM0);
            c.frontRightName.set(Constants.CHM2);
            c.backLeftName.set(Constants.CHM1);
            c.backRightName.set(Constants.CHM3);

            c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

            // Brake in manual/teleop mode so the robot stops when the sticks do.
            c.manualBrakeMode.set(true);
        });
    }

    /* ===================== Localizer: goBILDA Pinpoint ===================== */

    /**
     * TODO replace with the output of the <b>Pinpoint Tuner</b>.
     *
     * <p>What each field means, so the tuner's answers make sense:
     * <ul>
     *   <li>{@code name} — the Pinpoint's name in the robot configuration. It is
     *       an I2C device, so configure it as "goBILDA® Pinpoint Odometry
     *       Computer" on an I2C bus, not as a motor.</li>
     *   <li>{@code podType} — which goBILDA pods are fitted, which sets the
     *       encoder resolution. {@code goBILDA_4_BAR_POD} or
     *       {@code goBILDA_SWINGARM_POD}. For anything else, leave podType alone
     *       and set {@code ticksPerUnit} instead; the tuner derives that by having
     *       you push the robot a measured distance.</li>
     *   <li>{@code xPodOffset} — how far the <i>forward</i> (x) pod sits to the
     *       left of robot centre. Left is positive.</li>
     *   <li>{@code yPodOffset} — how far the <i>strafe</i> (y) pod sits forward of
     *       robot centre. Forward is positive.</li>
     *   <li>{@code xPodDirection} / {@code yPodDirection} — whether each pod
     *       counts up in the direction Pedro expects. The tuner has you push the
     *       robot forward, then left, and works these out.</li>
     * </ul>
     *
     * <p>The offsets are the part most often got wrong, and getting them wrong
     * does not look like a small error — it couples translation into heading and
     * sends the follower chasing a pose that is not where the robot is. Let the
     * tuner measure them.
     */
    public static PinpointConfig localizerConfig() {
        return new PinpointConfig(c -> {
            c.name.set("pinpoint");

            c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            // For non-goBILDA pods, drop podType and use the tuner's measured value:
            // c.ticksPerUnit.set(OptionalDouble.of(13.26291192));

            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);

            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);

            c.offsetUnits.set(DistanceUnit.INCH);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
        });
    }

    /* ===================== Following algorithm ===================== */

    /**
     * TODO replace with the output of the <b>Foresight Tuner</b>.
     *
     * <p>Structured the way the tuner emits it: piecewise translational
     * controllers that switch from a secondary to a primary gain at 2.5 inches of
     * error, proportional feedforward for coast and brake, and a braking model
     * made of linear and quadratic coefficient matrices.
     *
     * <p>The numbers below are Pedro's stock figures, not measurements of this
     * robot. The four physical ones in particular —
     * {@code maxAchievableForwardVelocity}, {@code maxAchievableStrafeVelocity},
     * {@code naturalForwardDeceleration}, {@code naturalStrafeDeceleration} —
     * are what the whole feedforward and braking model is built on, so a path run
     * against stock values will overshoot or undershoot badly.
     *
     * <p>The brake coefficients have no hand-derivable value. Seeding the
     * quadratic term with {@code 1/(2a)} makes it predict exactly the coasting
     * distance {@code v^2/(2a)}, which is why the placeholders below are what they
     * are: physically sane, erring toward slight overshoot, which the
     * translational controllers then clean up. The tuner replaces them with
     * measured ones.
     */
    public static ForesightConfig foresightConfig() {
        final double forwardVelocity = 81.34056;
        final double strafeVelocity = 65.43028;
        final double forwardDeceleration = 34.62719;
        final double strafeDeceleration = 78.15554;

        return new ForesightConfig(c -> {
            Controller primaryTranslationalForward = Controller.proportional(0.1);
            Controller secondaryTranslationalForward = Controller.proportional(0.1);
            Controller primaryTranslationalLateral = Controller.proportional(0.1);
            Controller secondaryTranslationalLateral = Controller.proportional(0.1);

            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward)
                    .put(2.5, primaryTranslationalForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral)
                    .put(2.5, primaryTranslationalLateral));

            c.coast.set(Controller.proportionalFeedforward(0.015));
            c.brake.set(Controller.proportionalFeedforward(0.015));

            c.headingFeedback.set(Controller.proportional(1.0));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0, 0.0));

            c.linearBrakeCoefficients.set(Matrix.diag(0.0, 0.0));
            c.quadraticBrakeCoefficients.set(Matrix.diag(
                    1.0 / (2.0 * forwardDeceleration),
                    1.0 / (2.0 * strafeDeceleration)));

            c.maxAchievableForwardVelocity.set(forwardVelocity);
            c.maxAchievableStrafeVelocity.set(strafeVelocity);
            c.naturalForwardDeceleration.set(forwardDeceleration);
            c.naturalStrafeDeceleration.set(strafeDeceleration);
        });
    }

    /* ===================== Follower ===================== */

    /**
     * The three pieces a Follower is made of, each built from the configs above.
     *
     * <p>Split out because the tuning procedures need them individually, and they
     * take them in inconsistent orders — {@code ForesightTuner} wants
     * (localizer, drivetrain) while {@code Tests} wants (drivetrain, localizer,
     * algorithm). Method references to these cannot be transposed by mistake the
     * way two bare lambdas of the same shape can.
     */
    public static Localizer createLocalizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, localizerConfig());
    }

    public static Drivetrain createDrivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig());
    }

    public static Algorithm createAlgorithm() {
        return new Foresight(foresightConfig());
    }

    /**
     * Builds the Follower the robot drives with.
     *
     * <p>Pedro 3 dropped {@code FollowerBuilder}: a Follower is assembled directly
     * from a localizer, a drivetrain and an algorithm.
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                createLocalizer(hardwareMap),
                createDrivetrain(hardwareMap),
                createAlgorithm());
    }

    /**
     * Kept for compatibility with the Pedro quickstart, which names this
     * {@code create}.
     */
    public static Follower create(HardwareMap hardwareMap) {
        return createFollower(hardwareMap);
    }
}
