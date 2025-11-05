package org.firstinspires.ftc.team12841.teleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.team12841.RobotHardware;
import org.firstinspires.ftc.team12841.pedroPathing.Constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

@TeleOp(name = "TeleOp Pathing", group = "opModes")
public class teleOpPathing extends OpMode {

    private RobotHardware robot;
    private Follower follower;

    private double headingOffset = 0;
    private boolean babyMode = false;
    private final double BABY_MODE_VAL = 0.4;

    @Override
    public void init() {
        robot = new RobotHardware(this);
        telemetry.addLine("Initialized");

        // Initialize Pedro Follower (hardwareMap passed inside RobotHardware)
        follower = Constants.createFollower(hardwareMap);  // Create follower per your constants setup
        // Set starting pose (optional) — e.g., Pose(0,0,0)
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleopDrive();

        // Reset heading offset if needed
        headingOffset = follower.getPose().getHeading();
    }

    @Override
    public void loop() {
        // Always update the follower
        follower.update();

        // Read joystick inputs
        double x = gamepad1.left_stick_x;      // strafe
        double y = -gamepad1.left_stick_y;     // forward/back (inverted)
        double rotation = gamepad1.right_stick_x;

        // Mode toggles
        if (gamepad1.right_bumper) babyMode = true;
        if (gamepad1.b) babyMode = false;
        if (gamepad1.a) {
            // Reset heading offset
            headingOffset = follower.getPose().getHeading();
        }

        // Apply baby mode scaling
        double speedScale = babyMode ? BABY_MODE_VAL : 1.0;
        x *= speedScale;
        y *= speedScale;
        rotation *= speedScale;

        // Teleop drive call from Pedro
        // The last argument indicates robotCentric: true = robot-centric, false = field-centric
        follower.setTeleOpDrive(
                y,
                x,
                rotation,
                false   // field-centric
        );

        // Telemetry
        Pose pose = follower.getPose();
        telemetry.addData("Pose X", "%.2f", pose.getX());
        telemetry.addData("Pose Y", "%.2f", pose.getY());
        telemetry.addData("Heading (°)", "%.2f", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Baby Mode", babyMode);
        telemetry.update();
    }
}
