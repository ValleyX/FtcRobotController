package org.firstinspires.ftc.team12841;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.graphics.Color;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.localizers.ThreeWheelLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardware{
    OpMode opMode_;
    private final ElapsedTime runtime = new ElapsedTime();

    // Motors
    public DcMotorEx rfMotor = null;
    public DcMotorEx rbMotor = null;
    public DcMotorEx lfMotor = null;
    public DcMotorEx lbMotor = null;

    // Sensors
    IMU imu;
    public Limelight3A limelight3A = null;

    // Odometry Wheels

    public GoBildaPinpointDriver.GoBildaOdometryPods rightLocalizer = null;
    public GoBildaPinpointDriver.GoBildaOdometryPods strafeLocalizer = null;
    public GoBildaPinpointDriver.GoBildaOdometryPods leftLocalizer = null;

    public RobotHardware(OpMode opMode) {

        opMode_ = opMode; // I think everything dies if you delete this

        // Motor HardwareMaps

        rfMotor = opMode_.hardwareMap.get(DcMotorEx.class, "rfMotor");
        rbMotor = opMode_.hardwareMap.get(DcMotorEx.class, "rbMotor");
        lfMotor = opMode_.hardwareMap.get(DcMotorEx.class, "lfMotor");
        lbMotor = opMode_.hardwareMap.get(DcMotorEx.class, "lbMotor");

        // Odometry HardwareMaps

        rightLocalizer = opMode_.hardwareMap.get(GoBildaPinpointDriver.GoBildaOdometryPods.class, "rbMotor"); // Right Odo
        strafeLocalizer = opMode_.hardwareMap.get(GoBildaPinpointDriver.GoBildaOdometryPods.class, "rfMotor"); // Strafe Odo
        leftLocalizer = opMode_.hardwareMap.get(GoBildaPinpointDriver.GoBildaOdometryPods.class, "lbMotor"); // Left Odo

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        lbMotor.setDirection(DcMotor.Direction.REVERSE);
        lfMotor.setDirection(DcMotor.Direction.REVERSE);
        rbMotor.setDirection(DcMotor.Direction.FORWARD);
        rfMotor.setDirection(DcMotor.Direction.FORWARD);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        rbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Sensor HardwareMaps

        limelight3A = opMode_.hardwareMap.get(Limelight3A.class, "limelight");

        imu = opMode_.hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

    }
}