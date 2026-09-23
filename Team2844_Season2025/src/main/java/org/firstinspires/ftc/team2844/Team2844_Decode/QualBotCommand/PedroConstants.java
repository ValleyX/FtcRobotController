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
     * <p>TODO: verify the three encoder directions on the robot. Road Runner
     * read the encoders raw, while Pedro folds in the direction of the motor the
     * encoder is plugged into, so the RR reversals below do not necessarily
     * carry over one-for-one. Push the robot forward and confirm x grows; strafe
     * left and confirm y grows.
     */
    public static ThreeWheelIMUConstants localizerConstants() {
        return new ThreeWheelIMUConstants()
                .forwardTicksToInches(IN_PER_TICK)
                .strafeTicksToInches(IN_PER_TICK)
                .turnTicksToInches(IN_PER_TICK)
                .leftPodY(LEFT_POD_Y)
                .rightPodY(RIGHT_POD_Y)
                .strafePodX(STRAFE_POD_X)
                .leftEncoder_HardwareMapName(RobotConstants.LEFT_FRONT_MOTOR)
                .rightEncoder_HardwareMapName(RobotConstants.RIGHT_FRONT_MOTOR)
                .strafeEncoder_HardwareMapName(RobotConstants.RIGHT_BACK_MOTOR)
                .leftEncoderDirection(Encoder.REVERSE)
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
