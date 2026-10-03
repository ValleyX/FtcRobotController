package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.ThreeWheelIMUConfig;
import com.pedropathing.revhub.localizers.ThreeWheelIMULocalizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Pedro Pathing 3 configuration for the Qual Bot: mecanum drivetrain with a
 * three dead wheel + IMU localizer.
 *
 * <p>The odometry geometry was converted from the team's Road Runner
 * {@code ThreeDeadWheelLocalizer} tuning, so the pod offsets and ticks-per-inch
 * start from numbers already measured on this robot.
 *
 * <h2>Coordinates</h2>
 * Pedro 3 dropped the coordinate-system abstraction: a {@link com.pedropathing.math.Pose}
 * is a bare (x, y, heading) and the frame is simply whatever the localizer is
 * given at startup. So everything here stays in the FTC-standard centre-origin
 * frame the old Road Runner autos were written in, with no conversion at all --
 * the waypoints in {@code autos/} are the same literals they always were.
 */
public final class PedroConstants {

    private PedroConstants() {}

    /* ===================== Odometry geometry ===================== */

    /** Inches of travel per encoder tick, from Road Runner's {@code inPerTick}. */
    public static final double IN_PER_TICK = 70.0 / 33782.167;

    /**
     * Pod offsets, converted from the Road Runner tick-space values by scaling
     * each one by {@link #IN_PER_TICK}.
     *
     * <ul>
     *   <li>{@code par1YTicks = 2199.826} (leftFront port) -&gt; left pod</li>
     *   <li>{@code par0YTicks = -2041.520} (rightFront port) -&gt; right pod</li>
     *   <li>{@code perpXTicks = -3417.406} (rightBack port) -&gt; strafe pod</li>
     * </ul>
     */
    public static final double LEFT_POD_Y = 2199.826182562763 * IN_PER_TICK;
    public static final double RIGHT_POD_Y = -2041.5202793245392 * IN_PER_TICK;
    public static final double STRAFE_POD_X = -3417.4060854286877 * IN_PER_TICK;

    /** Separation between the two parallel pods, i.e. the odometry track width. */
    public static final double POD_SEPARATION = LEFT_POD_Y - RIGHT_POD_Y;

    /* ===================== Measured robot performance ===================== */

    /**
     * TODO: these four are Pedro's stock figures, not measured on this robot.
     * Run Pedro's velocity and zero-power-deceleration tuners and replace them.
     * Until then paths will be followed, but the feedforward and the braking
     * model will be off.
     */
    public static final double MAX_FORWARD_VELOCITY = 81.34056;
    public static final double MAX_STRAFE_VELOCITY = 65.43028;
    public static final double NATURAL_FORWARD_DECELERATION = 34.62719;
    public static final double NATURAL_STRAFE_DECELERATION = 78.15554;

    /* ===================== Drivetrain ===================== */

    /**
     * Motor names and directions match the old {@code RobotHardware}: both left
     * motors reversed, both right forward.
     *
     * <p>Pedro 3 renamed these from {@code leftFront}/{@code leftRear} to
     * {@code frontLeft}/{@code backLeft}, and configuration moved from chained
     * setters to a {@code Configuration} lambda over {@code ConfigVar} fields.
     */
    public static MecanumConfig mecanumConfig() {
        return new MecanumConfig(c -> {
            c.frontLeftName.set(RobotConstants.LEFT_FRONT_MOTOR);
            c.backLeftName.set(RobotConstants.LEFT_BACK_MOTOR);
            c.frontRightName.set(RobotConstants.RIGHT_FRONT_MOTOR);
            c.backRightName.set(RobotConstants.RIGHT_BACK_MOTOR);

            c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

            c.manualBrakeMode.set(true);
        });
    }

    /* ===================== Localizer ===================== */

    /**
     * Three dead wheel + IMU localizer.
     *
     * <p>Encoders are on the same motor ports Road Runner used: left pod on
     * {@code leftFront}, right pod on {@code rightFront}, strafe pod on
     * {@code rightBack}.
     *
     * <p><b>The encoder directions are deliberately not Road Runner's values.</b>
     * Road Runner's {@code RawEncoder} reads ticks raw; Pedro's {@link Encoder}
     * multiplies by the direction of the <em>motor</em> the encoder is plugged
     * into ({@code multiplier * (motor.getDirection() == FORWARD ? 1 : -1)}).
     * Because the drivetrain above reverses both left motors, copying Road
     * Runner's reversals across double-negates the left pod.
     *
     * <p>The effective sign each pod needs is left {@code -1}, right {@code -1},
     * strafe {@code +1}. Working back through the motor directions gives
     * FORWARD / REVERSE / FORWARD below.
     *
     * <p>Getting this wrong is not a small error: forward travel is a positively
     * weighted sum of the two parallel pods, so an inverted left pod nearly
     * cancels it (about 3.7% of actual) while injecting a false strafe of roughly
     * 1.6x the real distance. The follower then sees a robot making no progress
     * and drives at full power indefinitely. Verify on the robot before running a
     * path: push it forward a known distance and check the reported x matches.
     */
    public static ThreeWheelIMUConfig localizerConfig() {
        return new ThreeWheelIMUConfig(c -> {
            c.leftEncoderName.set(RobotConstants.LEFT_FRONT_MOTOR);
            c.rightEncoderName.set(RobotConstants.RIGHT_FRONT_MOTOR);
            c.strafeEncoderName.set(RobotConstants.RIGHT_BACK_MOTOR);
            c.imuName.set(RobotConstants.IMU);

            c.leftPodY.set(LEFT_POD_Y);
            c.rightPodY.set(RIGHT_POD_Y);
            c.strafePodX.set(STRAFE_POD_X);

            c.forwardTicksToInches.set(IN_PER_TICK);
            c.strafeTicksToInches.set(IN_PER_TICK);
            // Pedro 3 names this honestly: ticks to *radians*, where Pedro 2 called
            // the same field turnTicksToInches. One tick on one pod rotates the
            // robot by IN_PER_TICK over the pod separation. Only used if the IMU
            // is unavailable.
            c.turnTicksToRadians.set(IN_PER_TICK / POD_SEPARATION);

            c.leftEncoderDirection.set(Encoder.FORWARD);
            c.rightEncoderDirection.set(Encoder.REVERSE);
            c.strafeEncoderDirection.set(Encoder.FORWARD);

            c.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        });
    }

    /* ===================== Following algorithm ===================== */

    /**
     * Foresight, Pedro 3's path-following algorithm.
     *
     * <p>Pedro 3 replaced {@code FollowerConstants} and {@code PathConstraints}
     * with a single algorithm config built out of {@link Controller}s. The gains
     * below are Pedro 2's documented defaults carried across, so following starts
     * where the previous setup did rather than from nothing:
     *
     * <table>
     *   <tr><th>Pedro 2</th><th>value</th><th>Pedro 3</th></tr>
     *   <tr><td>translational PIDF</td><td>P 0.1, F 0.015</td><td>forward/strafeTranslational</td></tr>
     *   <tr><td>heading PIDF</td><td>P 1.0, F 0.01</td><td>headingFeedback + headingStaticFF</td></tr>
     *   <tr><td>drive PIDF</td><td>P 0.025, D 0.00001</td><td>brake and coast</td></tr>
     * </table>
     *
     * <p>The brake coefficients are new in Pedro 3 and have no Pedro 2 analogue.
     * Foresight models braking displacement as
     * {@code quadratic * v*|v| + linear * v}, so seeding the quadratic term with
     * {@code 1/(2a)} makes it predict exactly the coasting distance
     * {@code v^2/(2a)} -- a physically sane starting point that errs toward
     * slight overshoot, which the translational controller then cleans up.
     * Heading braking starts at zero, meaning no anticipation.
     *
     * <p>TODO: run Pedro 3's autotuner to replace the brake coefficients and
     * these gains with measured values.
     */
    public static ForesightConfig foresightConfig() {
        return new ForesightConfig(c -> {
            c.forwardTranslational.set(Controller.sum(
                    Controller.pid(0.1, 0.0, 0.0),
                    Controller.staticFeedforward(0.015)));
            c.strafeTranslational.set(Controller.sum(
                    Controller.pid(0.1, 0.0, 0.0),
                    Controller.staticFeedforward(0.015)));

            c.headingFeedback.set(Controller.pid(1.0, 0.0, 0.0));
            c.headingStaticFF.set(Controller.staticFeedforward(0.01));

            c.brake.set(Controller.pid(0.025, 0.0, 0.00001));
            c.coast.set(Controller.pid(0.025, 0.0, 0.00001));

            c.maxAchievableForwardVelocity.set(MAX_FORWARD_VELOCITY);
            c.maxAchievableStrafeVelocity.set(MAX_STRAFE_VELOCITY);
            c.naturalForwardDeceleration.set(NATURAL_FORWARD_DECELERATION);
            c.naturalStrafeDeceleration.set(NATURAL_STRAFE_DECELERATION);

            c.quadraticBrakeCoefficients.set(Matrix.diag(
                    1.0 / (2.0 * NATURAL_FORWARD_DECELERATION),
                    1.0 / (2.0 * NATURAL_STRAFE_DECELERATION)));
            c.linearBrakeCoefficients.set(Matrix.diag(0.0, 0.0));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0, 0.0));
        });
    }

    /** Builds the Follower this robot drives with, in both auto and teleop. */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new ThreeWheelIMULocalizer(hardwareMap, localizerConfig()),
                new Mecanum(hardwareMap, mecanumConfig()),
                new Foresight(foresightConfig()));
    }

    /* ===================== Pose factories ===================== */

    /**
     * FTC-standard field coordinates with headings in degrees, no alliance
     * transform: centre origin, +-72 inches, +x toward the red wall, +y to its
     * left.
     *
     * <p>Use this where the waypoints are already written out per alliance, as the
     * far-goal autos are -- passing those through {@link #RED} would mirror them a
     * second time.
     */
    public static final PoseFactory FIELD = PoseFactory.degrees();

    /** Blue alliance: the field frame exactly as written. */
    public static final PoseFactory BLUE = FIELD;

    /**
     * Red alliance: the blue frame mirrored across the centre line.
     *
     * <p>{@code mirrorY(0)} maps {@code (x, y, h)} to {@code (x, -y, -h)}, which
     * is exactly the blue-to-red mirror the autos used to apply by hand. Only use
     * it with poses written in blue coordinates.
     */
    public static final PoseFactory RED = FIELD.mirrorY(0);
}
