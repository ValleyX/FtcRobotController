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

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.25)
            .forwardZeroPowerAcceleration(-22.915006748599247)
            .lateralZeroPowerAcceleration(37.86918694342776)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.03, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03,0.0,0.001,0.6,0.01));

    public static PathConstraints pathConstraints =
            new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rfMotor")
            .rightRearMotorName("rbMotor")
            .leftRearMotorName("lbMotor")
            .leftFrontMotorName("lfMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(84.31585884534567)
            .yVelocity(60.51963513206436);

    public static ThreeWheelIMUConstants localizerConstants =
            new ThreeWheelIMUConstants()
                    .forwardTicksToInches(-0.0021395637433314642)
                    .strafeTicksToInches(0.0021868047299924316)
                    .turnTicksToInches(0.0121104)

                    .leftPodY(-2.5)
                    .rightPodY(3.75)
                    .strafePodX(6.3)

                    // ODOMETRY NAMES
                    .leftEncoder_HardwareMapName("leftOdo")
                    .rightEncoder_HardwareMapName("rightOdo")
                    .strafeEncoder_HardwareMapName("strafeOdo")

                    // Directions
                    .leftEncoderDirection(Encoder.FORWARD)
                    .rightEncoderDirection(Encoder.FORWARD)
                    .strafeEncoderDirection(Encoder.FORWARD)

                    // IMU orientation
                    .IMU_HardwareMapName("imu")
                    .IMU_Orientation(new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP));

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelIMULocalizer(localizerConstants)
                .build();
    }
}
