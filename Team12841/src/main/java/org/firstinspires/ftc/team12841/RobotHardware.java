package org.firstinspires.ftc.team12841;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team12841.configs.PanelsConfig;
import org.firstinspires.ftc.team12841.pedroPathing.Constants;

public class RobotHardware {

    /* ===================== CORE ===================== */

    public final OpMode opMode;
    private Follower follower;

    /* ===================== CONSTANTS ===================== */

    public static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double INVALID = -999;

    /* ===================== MOTORS ===================== */

    public DcMotorEx lf, lb, rf, rb;
    public DcMotorEx shooter, intake, flick;
    public DcMotorEx leftOdo, rightOdo, strafeOdo;

    /* ===================== SENSORS ===================== */

    public IMU imu;
    public Limelight3A limelight;
    private LLResult llResult;
    private DigitalChannel beamBreak;
    public Servo light;

    /* ===================== DRIVE STATE ===================== */

    private double lfDrive, lbDrive, rfDrive, rbDrive;
    private double lfAlign, lbAlign, rfAlign, rbAlign;

    /* ===================== SHOOTER STATE ===================== */

    public static double rpm = 2400;

    /* ===================== INIT ===================== */

    public RobotHardware(OpMode opMode) {
        this.opMode = opMode;

        mapHardware();
        configureMotors();
        initIMU();
        initLimelight();

        follower = Constants.createFollower(opMode.hardwareMap);
        limelight.pipelineSwitch(0);
    }

    private void mapHardware() {
        lf = motor("lfMotor");
        lb = motor("lbMotor");
        rf = motor("rfMotor");
        rb = motor("rbMotor");

        shooter = motor("shooterMotor");
        intake  = motor("intakeMotor");
        flick   = motor("flickMotor");

        leftOdo   = lf;
        rightOdo  = rb;
        strafeOdo = lb;

        beamBreak = opMode.hardwareMap.get(DigitalChannel.class, "distanceSensor");
    }

    private DcMotorEx motor(String name) {
        return opMode.hardwareMap.get(DcMotorEx.class, name);
    }

    private void configureMotors() {
        lf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{lf, lb, rf, rb}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(500, 0, 0, 10);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flick.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        resetOdometry();
    }

    private void resetOdometry() {
        for (DcMotorEx odo : new DcMotorEx[]{leftOdo, rightOdo, strafeOdo}) {
            odo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            odo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void initIMU() {
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
    }

    private void initLimelight() {
        try {
            limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }
        light = opMode.hardwareMap.get(Servo.class, "light");
    }

    /* ===================== DRIVE ===================== */

    public void addAlign(double lf, double lb, double rf, double rb) {
        lfAlign = lf;
        lbAlign = lb;
        rfAlign = rf;
        rbAlign = rb;
        applyDrive();
    }

    private void applyDrive() {
        lf.setPower(clamp(lfDrive + lfAlign));
        lb.setPower(clamp(lbDrive + lbAlign));
        rf.setPower(clamp(rfDrive + rfAlign));
        rb.setPower(clamp(rbDrive + rbAlign));
    }

    private double clamp(double v) {
        return Math.max(-1.0, Math.min(1.0, v));
    }

    /* ===================== IMU ===================== */

    public double headingDeg() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading() {
        imu.resetYaw();
        Pose p = follower.getPose();
        follower.setPose(new Pose(p.getX(), p.getY(), 0));
    }

    /* ===================== LIMELIGHT ===================== */

    private void updateLL() {
        if (limelight != null) {
            llResult = limelight.getLatestResult();
        }
    }

    public double getTx() {
        updateLL();
        return (llResult != null && llResult.isValid()) ? -llResult.getTx() : INVALID;
    }

    public double getDistance() {
        updateLL();
        return (llResult != null && llResult.isValid())
                ? 68.86747 * Math.pow(llResult.getTa(), -0.5169279)
                : INVALID;
    }

    public void alignWithLimelight(double speed) {
        double tx = getTx();
        if (tx == INVALID) {
            addAlign(0, 0, 0, 0);
            return;
        }
        double power = -(tx * PanelsConfig.LLPGAIN * speed);
        addAlign(-power, -power, power, power);
    }

    /* ===================== SHOOTER ===================== */

    public void setShooterRPM(double rpm) {
        shooter.setVelocity((rpm * SHOOTER_TICKS_PER_REV) / 60.0);
    }

    public void stopShooter() {
        shooter.setVelocity(0);
    }

    public double calculateRegression() {
        double x = getDistance();
        if (x == INVALID) return rpm;

        rpm =
                (0.0000787632 * Math.pow(x, 4)) -
                        ( 0.0228824   * Math.pow(x, 3)) +
                        ( 2.35453    * Math.pow(x, 2)) -
                        ( 96.04063     * x) +
                        3742.94414;
        return rpm;
        // y=0.0000787632x^{4}-0.0228824x^{3}+2.35453x^{2}-96.04063x+3742.94414
    }

    /* ===================== MISC ===================== */

    public boolean isBroken() {
        return beamBreak.getState();
    }

    public Follower getFollower() {
        return follower;
    }

    public void setGoBildaLight(double color)
    {
        light.setPosition(color);
    }
}