package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PedroConstants {

    // Motor names match what MecanumDrive.java actually maps:
    //   RR "leftFront" field <- hardwaremap "leftBack", etc.
    //   Pedro just reads the hardware map directly, so we use the config names as-is.
    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("leftBack")
            .leftRearMotorName("leftFront")
            .rightFrontMotorName("rightBack")
            .rightRearMotorName("rightFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            // TODO: run ForwardVelocityTuner and StrafeVelocityTuner to get real values
            .xVelocity(60.0)
            .yVelocity(50.0);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")  // matches Constants.CBUS1
            // TODO: measure pod offsets from center of rotation in inches
            .forwardPodY(0.0)
            .strafePodX(0.0)
            // TODO: run LocalizationTest and flip these if x/y go the wrong direction
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static FollowerConstants followerConstants = new FollowerConstants()
            // TODO: weigh your robot in kg
            .mass(10.0)
            // TODO: run ZeroPowerAccelerationTuner to get real values
            .forwardZeroPowerAcceleration(-30.0)
            .lateralZeroPowerAcceleration(-60.0)
            // PIDF defaults — tune after localizer is verified
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(1.0, 0, 0, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.00035, 0.6, 0.015))
            .centripetalScaling(0.0005);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995, 0.1, 0.1, 0.009, 50, 1.25, 10, 1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}