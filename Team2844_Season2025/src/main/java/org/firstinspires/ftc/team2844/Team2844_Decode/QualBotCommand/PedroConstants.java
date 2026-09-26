package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Pedro Pathing configuration for the Qual Bot: mecanum drivetrain with a
 * three dead wheel + IMU localizer.
 *
 * <p>The odometry geometry here was converted from the team's Road Runner
 * {@code ThreeDeadWheelLocalizer} tuning, so the pod offsets and ticks-per-inch
 * start from numbers that were already measured on this robot.
 */
public final class PedroConstants {

    private PedroConstants() {}

    /* ===================== Odometry geometry ===================== */

    /**
     * Inches of travel per encoder tick.
     *
     * <p>Straight from Road Runner's {@code MecanumDrive.PARAMS.inPerTick}.
     */
    public static final double IN_PER_TICK = 70.0 / 33782.167;

    /**
     * Pod offsets, converted from the Road Runner tick-space values.
     *
     * <p>RR stored these as tick positions; Pedro wants inches, so each one is
     * just the RR value scaled by {@link #IN_PER_TICK}.
     * <ul>
     *   <li>{@code par1YTicks = 2199.826} (leftFront port) -> left pod</li>
     *   <li>{@code par0YTicks = -2041.520} (rightFront port) -> right pod</li>
     *   <li>{@code perpXTicks = -3417.406} (rightBack port) -> strafe pod</li>
     * </ul>
     */
    public static final double LEFT_POD_Y = 2199.826182562763 * IN_PER_TICK;
    public static final double RIGHT_POD_Y = -2041.5202793245392 * IN_PER_TICK;
    public static final double STRAFE_POD_X = -3417.4060854286877 * IN_PER_TICK;

    /* ===================== Follower construction ===================== */

    /**
     * Drivetrain constants.
     *
     * <p>Motor names and directions match the old {@code RobotHardware}: both
     * left motors reversed, both right motors forward.
     *
     * <p>TODO: {@code xVelocity} / {@code yVelocity} are still Pedro's stock
     * numbers. Run Pedro's Forward and Lateral Velocity tuners on this robot and
     * drop the measured values in — until then paths will be followed, but the
     * feedforward will be off.
     */
    public static MecanumConstants mecanumConstants() {
        return new MecanumConstants()
                .leftFrontMotorName(RobotConstants.LEFT_FRONT_MOTOR)
                .leftRearMotorName(RobotConstants.LEFT_BACK_MOTOR)
                .rightFrontMotorName(RobotConstants.RIGHT_FRONT_MOTOR)
                .rightRearMotorName(RobotConstants.RIGHT_BACK_MOTOR)
                .leftFrontMotorDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE)
                .leftRearMotorDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE)
                .rightFrontMotorDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD)
                .rightRearMotorDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD)
                .useBrakeModeInTeleOp(true)
                .xVelocity(81.34056)
                .yVelocity(65.43028);
    }

    /**
     * Three dead wheel + IMU localizer constants.
     *
     * <p>Encoders are plugged into the same motor ports Road Runner used:
     * left pod on {@code leftFront}, right pod on {@code rightFront},
     * strafe pod on {@code rightBack}.
     *
     * <p><b>Encoder directions are not the Road Runner values.</b> Road Runner's
     * {@code RawEncoder} reads ticks raw, but Pedro's {@link Encoder} multiplies
     * by the direction of the <em>motor</em> the encoder is plugged into:
     * {@code getMultiplier() = direction * (motor.getDirection() == FORWARD ? 1 : -1)}.
     * Since the drivetrain config below reverses both left motors, copying Road
     * Runner's reversals across double-negates the left pod.
     *
     * <p>The effective sign each pod needs, to match the tuning the pod offsets
     * came from, is left {@code -1}, right {@code -1}, strafe {@code +1}. Working
     * back through the motor directions:
     * <table>
     *   <tr><th>pod</th><th>port</th><th>motor dir</th><th>needed cfg</th></tr>
     *   <tr><td>left</td><td>leftFront</td><td>REVERSE (-1)</td><td>FORWARD</td></tr>
     *   <tr><td>right</td><td>rightFront</td><td>FORWARD (+1)</td><td>REVERSE</td></tr>
     *   <tr><td>strafe</td><td>rightBack</td><td>FORWARD (+1)</td><td>FORWARD</td></tr>
     * </table>
     *
     * <p>Getting this wrong is not a small error. The localizer derives forward
     * travel from a positively weighted sum of the two parallel pods, so an
     * inverted left pod very nearly cancels it: forward reads about 3.7% of
     * actual, and straight-line driving also injects a false strafe of roughly
     * 1.6x the real travel. The follower then sees a robot that never makes
     * progress and drives at full power indefinitely. Verify on the robot before
     * running a path: push it forward a known distance and check the reported x
     * matches, then strafe left and check y.
     */
    public static ThreeWheelIMUConstants localizerConstants() {
        return new ThreeWheelIMUConstants()
                .forwardTicksToInches(IN_PER_TICK)
                .strafeTicksToInches(IN_PER_TICK)
                // Despite the name, this localizer uses this value as ticks to
                // *radians* (TURN_TICKS_TO_RADIANS = constants.turnTicksToInches),
                // and only when the IMU is unavailable. One tick on one pod turns
                // the robot by IN_PER_TICK over the pod separation.
                .turnTicksToInches(IN_PER_TICK / (LEFT_POD_Y - RIGHT_POD_Y))
                .leftPodY(LEFT_POD_Y)
                .rightPodY(RIGHT_POD_Y)
                .strafePodX(STRAFE_POD_X)
                .leftEncoder_HardwareMapName(RobotConstants.LEFT_FRONT_MOTOR)
                .rightEncoder_HardwareMapName(RobotConstants.RIGHT_FRONT_MOTOR)
                .strafeEncoder_HardwareMapName(RobotConstants.RIGHT_BACK_MOTOR)
                .leftEncoderDirection(Encoder.FORWARD)
                .rightEncoderDirection(Encoder.REVERSE)
                .strafeEncoderDirection(Encoder.FORWARD)
                .IMU_HardwareMapName(RobotConstants.IMU)
                .IMU_Orientation(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
    }

    /**
     * Path following gains and robot physical properties.
     *
     * <p>TODO: {@code mass}, {@code forwardZeroPowerAcceleration} and
     * {@code lateralZeroPowerAcceleration} are Pedro defaults. Weigh the robot
     * and run the zero-power-acceleration tuners.
     */
    public static FollowerConstants followerConstants() {
        return new FollowerConstants()
                .mass(10.65)
                .forwardZeroPowerAcceleration(-34.62719)
                .lateralZeroPowerAcceleration(-78.15554)
                .centripetalScaling(0.0005);
    }

    /** Builds the Follower this robot drives with, in both auto and teleop. */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants(), hardwareMap)
                .mecanumDrivetrain(mecanumConstants())
                .threeWheelIMULocalizer(localizerConstants())
                .pathConstraints(PathConstraints.defaultConstraints)
                .build();
    }

    /* ===================== Coordinates ===================== */

    /**
     * Builds a Pedro {@link Pose} from FTC-standard field coordinates — the
     * centre-origin, +-72 inch frame the team's Road Runner autos were written in.
     *
     * <p>Every pose in this package is written in those coordinates and converted
     * here, so the waypoints stay recognisable against the old autos.
     *
     * @param x       inches, +x toward the red alliance wall
     * @param y       inches, +y to the left of +x
     * @param heading radians, 0 pointing along +x
     */
    public static Pose ftcPose(double x, double y, double heading) {
        return new Pose(x, y, heading, FTCCoordinates.INSTANCE)
                .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }

    /** {@link #ftcPose(double, double, double)} with a heading of 0. */
    public static Pose ftcPose(double x, double y) {
        return ftcPose(x, y, 0.0);
    }
}
