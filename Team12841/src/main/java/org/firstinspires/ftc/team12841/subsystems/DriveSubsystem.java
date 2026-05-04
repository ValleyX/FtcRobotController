package org.firstinspires.ftc.team12841.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.vcs.valleylib.core.subsystem.Subsystem;

public class DriveSubsystem extends Subsystem {

    private final DcMotorEx lf, lb, rf, rb;
    private final IMU imu;
    private final Follower follower;

    private double lfAlign, lbAlign, rfAlign, rbAlign;

    public DriveSubsystem(HardwareMap hw, Follower follower) {
        this.follower = follower;

        lf = hw.get(DcMotorEx.class, "lfMotor");
        lb = hw.get(DcMotorEx.class, "lbMotor");
        rf = hw.get(DcMotorEx.class, "rfMotor");
        rb = hw.get(DcMotorEx.class, "rbMotor");

        configure();

        imu = hw.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
    }

    private void configure() {
        lf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{lf, lb, rf, rb}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void drive(double lfP, double lbP, double rfP, double rbP) {
        lf.setPower(clamp(lfP + lfAlign));
        lb.setPower(clamp(lbP + lbAlign));
        rf.setPower(clamp(rfP + rfAlign));
        rb.setPower(clamp(rbP + rbAlign));
    }

    public void addAlign(double lf, double lb, double rf, double rb) {
        lfAlign = lf;
        lbAlign = lb;
        rfAlign = rf;
        rbAlign = rb;
    }

    public double headingDeg() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading() {
        imu.resetYaw();
        Pose p = follower.getPose();
        follower.setPose(new Pose(p.getX(), p.getY(), 0));
    }

    private double clamp(double v) {
        return Math.max(-1.0, Math.min(1.0, v));
    }
}