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
            .mass(13.4) // Weigh Robot in KG
            .forwardZeroPowerAcceleration(-32.20906457774129) // Tuning OpMode
            .lateralZeroPowerAcceleration(-39.73511957623294) // Tuning OpMode

            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.00001, 0.6, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients( 1, 0, 0, 0.01));

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
            .xVelocity(125.14010) // Tuning OpMode
            .yVelocity(50.3566989); // Tuning OpMode


    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants() // This changes based on whether you use 2 odometry wheels or 3, see docs
            .forwardTicksToInches(0.00197913724762) // Tuning OpMode (take average of 5ish runs)
            .strafeTicksToInches(0.001195489955537) // Tuning OpMode (take average of 5ish runs)
            .turnTicksToInches(0.014324695549060188) // Tuning OpMode (take average of 5ish runs)
            .leftPodY(3.5) // From center in inches, chart on docs
            .rightPodY(-2.3) // From center in inches, chart on docs
            .strafePodX(-5.75) // From center in inches, chart on docs
            .leftEncoder_HardwareMapName("lfMotor")
            .rightEncoder_HardwareMapName("rbMotor")
            .strafeEncoder_HardwareMapName("lbMotor")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelIMULocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}