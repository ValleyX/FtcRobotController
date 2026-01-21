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

    /* ===================== ROBOT PHYSICAL PROPERTIES ===================== */

    public static final double ROBOT_MASS_KG = 12.05;

    public static final double FORWARD_ZERO_POWER_ACCEL = 0;   // placeholder
    public static final double LATERAL_ZERO_POWER_ACCEL = 0;   // placeholder

    /* ===================== PIDF COEFFICIENTS ===================== */

    public static final double TRANSLATIONAL_P = 8.0; // placeholder
    public static final double TRANSLATIONAL_I = 0.0;// placeholder// placeholder
    public static final double TRANSLATIONAL_D = 0.4;// placeholder
    public static final double TRANSLATIONAL_F = 0.0;// placeholder

    public static final double HEADING_P = 6.0;// placeholder
    public static final double HEADING_I = 0.0;// placeholder
    public static final double HEADING_D = 0.3;// placeholder
    public static final double HEADING_F = 0.0;// placeholder

    /* ===================== PATH CONSTRAINTS ===================== */

    public static final double MAX_PATH_POWER   = 1.0;
    public static final double MAX_PATH_VELOCITY = 60.0; // in/s
    public static final double MAX_PATH_ACCEL    = 60.0; // in/s^2
    public static final double MAX_PATH_DECEL    = 60.0; // in/s^2

    /* ===================== MECANUM VELOCITIES ===================== */

    public static final double MAX_X_VELOCITY = 60.0;
    public static final double MAX_Y_VELOCITY = 60.0;

    /* ===================== ODOMETRY CONVERSIONS ===================== */

    public static final double FORWARD_TICKS_TO_INCHES = 0.000975; // placeholder
    public static final double STRAFE_TICKS_TO_INCHES  = 0.000975; // placeholder
    public static final double TURN_TICKS_TO_INCHES    = 0.000975; // placeholder

    /* ===================== ODOMETRY POD POSITIONS ===================== */

    public static final double LEFT_POD_Y   = 7.0;// placeholder  16.8 total 5.1 apart
    public static final double RIGHT_POD_Y  = -7.0;// placeholder
    public static final double STRAFE_POD_X = -6.0;// placeholder

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
                                    0.02,                 // T = derivative filter time constant
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

                    .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
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

                    .leftEncoder_HardwareMapName("intakeMotor")
                    .rightEncoder_HardwareMapName("rightOdo")
                    .strafeEncoder_HardwareMapName("strafeOdo")

                    .leftEncoderDirection(Encoder.FORWARD)
                    .rightEncoderDirection(Encoder.REVERSE)
                    .strafeEncoderDirection(Encoder.REVERSE)

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
