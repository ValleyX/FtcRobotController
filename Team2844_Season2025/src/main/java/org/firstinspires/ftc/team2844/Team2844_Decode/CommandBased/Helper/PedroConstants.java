package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Pedro Pathing 3 setup for the CommandBased robot.
 *
 * <p>This robot only uses Pedro for field-centric teleop drive -- Road Runner
 * still owns localization for everything else -- so the only value here that
 * actually affects driving is the IMU orientation, which is what supplies the
 * heading the field-centric rotation uses. Pedro 3 made the pod geometry and
 * tick scalings required rather than defaulted, so they are filled in with
 * plausible values and flagged rather than left out.
 */
public class PedroConstants {

    /**
     * Motor names match what {@code MecanumDrive} maps: the Road Runner
     * "leftFront" field is wired to the hardware-map name "leftBack", and so on.
     * Pedro reads the hardware map directly, so the config names are used as-is.
     *
     * <p>Pedro 3 renamed these from {@code leftFront}/{@code leftRear} to
     * {@code frontLeft}/{@code backLeft}.
     */
    public static MecanumConfig driveConfig() {
        return new MecanumConfig(c -> {
            c.frontLeftName.set("leftBack");
            c.backLeftName.set("leftFront");
            c.frontRightName.set("rightBack");
            c.backRightName.set("rightFront");

            c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

            c.manualBrakeMode.set(true);
        });
    }

    /**
     * Two dead wheel + IMU localizer.
     *
     * <p>Only the IMU matters for field-centric drive. The pod offsets and tick
     * scalings below are required by Pedro 3 but are not used by anything this
     * robot does; TODO measure them if this follower is ever asked to follow a
     * path.
     */
    public static TwoWheelConfig localizerConfig() {
        return new TwoWheelConfig(c -> {
            c.xPodName.set("leftFront");
            c.yPodName.set("rightBack");
            c.imuName.set("imu");

            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);

            c.forwardTicksToInches.set(70.0 / 33782.167);
            c.strafeTicksToInches.set(70.0 / 33782.167);

            c.xPodDirection.set(Encoder.FORWARD);
            c.yPodDirection.set(Encoder.FORWARD);

            c.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        });
    }

    /**
     * Foresight, Pedro 3's path-following algorithm, replacing Pedro 2's
     * {@code FollowerConstants} plus {@code PathConstraints}.
     *
     * <p>None of these gains affect field-centric teleop, which is all this
     * follower is used for -- they are Pedro 2's defaults carried across so the
     * config is valid and would be a sane starting point if paths are ever added.
     */
    public static ForesightConfig foresightConfig() {
        final double naturalForwardDeceleration = 30.0;
        final double naturalStrafeDeceleration = 60.0;

        return new ForesightConfig(c -> {
            c.forwardTranslational.set(Controller.sum(
                    Controller.pid(0.1, 0.0, 0.0),
                    Controller.staticFeedforward(0.01)));
            c.strafeTranslational.set(Controller.sum(
                    Controller.pid(0.1, 0.0, 0.0),
                    Controller.staticFeedforward(0.01)));

            c.headingFeedback.set(Controller.pid(1.0, 0.0, 0.0));
            c.headingStaticFF.set(Controller.staticFeedforward(0.01));

            c.brake.set(Controller.pid(0.1, 0.0, 0.00035));
            c.coast.set(Controller.pid(0.1, 0.0, 0.00035));

            c.maxAchievableForwardVelocity.set(60.0);
            c.maxAchievableStrafeVelocity.set(50.0);
            c.naturalForwardDeceleration.set(naturalForwardDeceleration);
            c.naturalStrafeDeceleration.set(naturalStrafeDeceleration);

            // Predict the coasting distance v^2/(2a); see the QualBotCommand
            // PedroConstants for why this is a reasonable seed.
            c.quadraticBrakeCoefficients.set(Matrix.diag(
                    1.0 / (2.0 * naturalForwardDeceleration),
                    1.0 / (2.0 * naturalStrafeDeceleration)));
            c.linearBrakeCoefficients.set(Matrix.diag(0.0, 0.0));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0, 0.0));
        });
    }

    /**
     * Builds the Follower.
     *
     * <p>Pedro 3 dropped {@code FollowerBuilder}: a Follower is now assembled
     * directly from a localizer, a drivetrain and an algorithm.
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new TwoWheelLocalizer(hardwareMap, localizerConfig()),
                new Mecanum(hardwareMap, driveConfig()),
                new Foresight(foresightConfig()));
    }
}
