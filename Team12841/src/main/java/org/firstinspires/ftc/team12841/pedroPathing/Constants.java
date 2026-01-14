package org.firstinspires.ftc.team12841.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {


    // ---------- ROBOT PHYSICAL PROPERTIES ----------
    public static final double ROBOT_MASS_KG;

    public static final double FORWARD_ZERO_POWER_ACCEL;
    public static final double LATERAL_ZERO_POWER_ACCEL;

    // ---------- PIDF COEFFICIENTS ----------
    public static final double TRANSLATIONAL_P;
    public static final double TRANSLATIONAL_I;
    public static final double TRANSLATIONAL_D;
    public static final double TRANSLATIONAL_F;

    public static final double HEADING_P;
    public static final double HEADING_I;
    public static final double HEADING_D;
    public static final double HEADING_F;

    // ---------- PATH CONSTRAINTS ----------
    public static final double MAX_PATH_POWER;
    public static final double MAX_PATH_VELOCITY;
    public static final double MAX_PATH_ACCEL;
    public static final double MAX_PATH_DECEL;

    // ---------- MECANUM VELOCITIES ----------
    public static final double MAX_X_VELOCITY;
    public static final double MAX_Y_VELOCITY;

    // ---------- ODOMETRY CONVERSIONS ----------
    public static final double FORWARD_TICKS_TO_INCHES;
    public static final double STRAFE_TICKS_TO_INCHES;
    public static final double TURN_TICKS_TO_INCHES;

    // ---------- ODOMETRY POD POSITIONS ----------
    public static final double LEFT_POD_Y;
    public static final double RIGHT_POD_Y;
    public static final double STRAFE_POD_X;

    /* ===================== FOLLOWER CONSTANTS ===================== */

    public static final FollowerConstants followerConstants =
            new FollowerConstants()
                    .mass(ROBOT_MASS_KG)
                    .forwardZeroPowerAcceleration(FORWARD_ZERO_POWER_ACCEL)
                    .lateralZeroPowerAcceleration(LATERAL_ZERO_POWER_ACCEL)
                    .translationalPIDFCoefficients(
                            new PIDFCoefficients(
                                    TRANSLATIONAL_P,
                                    TRANSLATIONAL_I,
                                    TRANSLATIONAL_D,
                                    TRANSLATIONAL_F
                            )
                    )
                    .headingPIDFCoefficients(
                            new PIDFCoefficients(
                                    HEADING_P,
                                    HEADING_I,
                                    HEADING_D,
                                    HEADING_F
                            )
                    )
                    .drivePIDFCoefficients(
                            new FilteredPIDFCoefficients(
                                    TRANSLATIONAL_P,
                                    TRANSLATIONAL_I,
                                    TRANSLATIONAL_D,
                                    TRANSLATIONAL_F
                            )
                    );

    /* ===================== PATH CONSTRAINTS ===================== */

    public static final PathConstraints pathConstraints =
            new PathConstraints(
                    MAX_PATH_POWER,
                    MAX_PATH_VELOCITY,
                    MAX_PATH_ACCEL,
                    MAX_PATH_DECEL
            );

    /* ===================== MECANUM CONSTANTS ===================== */

    public static final MecanumConstants driveConstants =
            new MecanumConstants()
                    .maxPower(1.0)

                    .rightFrontMotorName("rfMotor")
                    .rightRearMotorName("rbMotor")
                    .leftRearMotorName("lbMotor")
                    .leftFrontMotorName("lfMotor")

                    .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

                    .xVelocity(MAX_X_VELOCITY)
                    .yVelocity(MAX_Y_VELOCITY);

    /* ===================== LOCALIZER CONSTANTS ===================== */

    public static final ThreeWheelIMUConstants localizerConstants =
            new ThreeWheelIMUConstants()
                    .forwardTicksToInches(FORWARD_TICKS_TO_INCHES)
                    .strafeTicksToInches(STRAFE_TICKS_TO_INCHES)
                    .turnTicksToInches(TURN_TICKS_TO_INCHES)

                    .leftPodY(LEFT_POD_Y)
                    .rightPodY(RIGHT_POD_Y)
                    .strafePodX(STRAFE_POD_X)

                    .leftEncoder_HardwareMapName("leftOdo")
                    .rightEncoder_HardwareMapName("rightOdo")
                    .strafeEncoder_HardwareMapName("strafeOdo")

                    .leftEncoderDirection(Encoder.FORWARD)
                    .rightEncoderDirection(Encoder.FORWARD)
                    .strafeEncoderDirection(Encoder.FORWARD)

                    .IMU_HardwareMapName("imu")
                    .IMU_Orientation(
                            new RevHubOrientationOnRobot(
                                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                    RevHubOrientationOnRobot.UsbFacingDirection.UP
                            )
                    );

    /* ===================== FOLLOWER FACTORY ===================== */

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelIMULocalizer(localizerConstants)
                .build();
    }
}
