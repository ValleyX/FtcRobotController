package org.firstinspires.ftc.team12841;

import android.graphics.Color;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.localizers.ThreeWheelLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardware {
    LinearOpMode opMode_;
    private final ElapsedTime runtime = new ElapsedTime();

    // Motors
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;

    // Odometry Wheels

    public GoBildaPinpointDriver.GoBildaOdometryPods rightLocalizer = null;
    public GoBildaPinpointDriver.GoBildaOdometryPods strafeLocalizer = null;
    public GoBildaPinpointDriver.GoBildaOdometryPods leftLocalizer = null;

    public RobotHardware(LinearOpMode opMode) {

        opMode_ = opMode; // I think everything dies if you delete this

        // Motor HardwareMaps

        rightFront = opMode_.hardwareMap.get(DcMotor.class, "rfMotor");
        rightBack = opMode_.hardwareMap.get(DcMotor.class, "rbMotor");
        leftFront = opMode_.hardwareMap.get(DcMotor.class, "lfMotor");
        leftBack = opMode_.hardwareMap.get(DcMotor.class, "lbMotor");

        // Odometry HardwareMaps

        rightLocalizer = opMode_.hardwareMap.get(GoBildaPinpointDriver.GoBildaOdometryPods.class, "rightLocalizer");
        strafeLocalizer = opMode_.hardwareMap.get(GoBildaPinpointDriver.GoBildaOdometryPods.class, "strafeLocalizer");
        leftLocalizer = opMode_.hardwareMap.get(GoBildaPinpointDriver.GoBildaOdometryPods.class, "leftLocalizer");
    }
        // Pedro Pathing Constants
        public static FollowerConstants followerConstants = new FollowerConstants()
                .mass(3.8);

        public static RobotHardware hardware = new RobotHardware();

        public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

        public static MecanumConstants driveConstants = new MecanumConstants()
                .maxPower(1)
                .rightFrontMotorName("rf")
                .rightRearMotorName("rr")
                .leftRearMotorName("lr")
                .leftFrontMotorName("lf")
                .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

        public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
                .forwardTicksToInches(.001989436789)
                .strafeTicksToInches(.001989436789)
                .turnTicksToInches(.001989436789)
                .leftPodY(1)
                .rightPodY(-1)
                .strafePodX(-2.5)
                .leftEncoder_HardwareMapName("leftLocalizer")
                .rightEncoder_HardwareMapName("rightLocalizer")
                .strafeEncoder_HardwareMapName("strafeLocalizer")
                .leftEncoderDirection(leftLocalizer.FORWARD)
                .rightEncoderDirection(rightLocalizer.FORWARD)
                .strafeEncoderDirection(strafeLocalizer.FORWARD);

        public static Follower createFollower(HardwareMap hardwareMap) {
            return new FollowerBuilder(followerConstants, hardwareMap)
                    .pathConstraints(pathConstraints)
                    .mecanumDrivetrain(driveConstants)
                    .build();
        }
    }
}