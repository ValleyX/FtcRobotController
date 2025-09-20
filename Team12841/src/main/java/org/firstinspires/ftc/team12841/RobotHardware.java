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
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    public DcMotorEx rfMotor = null;
    public DcMotorEx rbMotor = null;
    public DcMotorEx lfMotor = null;
    public DcMotorEx lbMotor = null;

    // Odometry Wheels

    public GoBildaPinpointDriver.GoBildaOdometryPods rightLocalizer = null;
    public GoBildaPinpointDriver.GoBildaOdometryPods strafeLocalizer = null;
    public GoBildaPinpointDriver.GoBildaOdometryPods leftLocalizer = null;

    public RobotHardware(LinearOpMode opMode) {

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
    }
}