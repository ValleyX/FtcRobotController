package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.vcs.valleylib.ftc.hardware.FtcSubsystem;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.RobotConstants;

/**
 * Flywheel, hood, and the gate that holds balls back from the flywheel.
 *
 * <p>Distance-to-velocity and distance-to-hood are the same linear regressions
 * the old {@code ShooterHardware} used.
 */
public class ShooterSubsystem extends FtcSubsystem {

    private final DcMotorEx shooterMotor;
    private final Servo blockServo;
    private final Servo hoodServo;

    /** Loosened in teleop, tightened in auto. */
    private double velocityThreshold = RobotConstants.VELOCITY_THRESHOLD_TELEOP;

    /** Last velocity/hood the regressions produced, for the "hold last shot" button. */
    private double lastKnownVelocity = RobotConstants.DEFAULT_VELOCITY;
    private double lastKnownHood = RobotConstants.HOOD_NEUTRAL;

    private boolean gateOpen = false;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        super(hardwareMap);

        shooterMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.SHOOTER_MOTOR);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setVelocityPIDFCoefficients(
                RobotConstants.SHOOTER_P,
                RobotConstants.SHOOTER_I,
                RobotConstants.SHOOTER_D,
                RobotConstants.SHOOTER_F);
        shooterMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(
                        RobotConstants.SHOOTER_P,
                        RobotConstants.SHOOTER_I,
                        RobotConstants.SHOOTER_D,
                        RobotConstants.SHOOTER_F));

        blockServo = hardwareMap.get(Servo.class, RobotConstants.BLOCK_SERVO);
        hoodServo = hardwareMap.get(Servo.class, RobotConstants.HOOD_SERVO);

        closeGate();
    }

    /**
     * Tightens the "at speed" window for auto, where there is time to wait for a
     * genuine spin-up instead of accepting anything in the neighbourhood.
     */
    public void useAutoVelocityThreshold() {
        velocityThreshold = RobotConstants.VELOCITY_THRESHOLD_AUTO;
    }

    /* ===================== Flywheel ===================== */

    /** Raw open-loop power, clamped to [-1, 1]. */
    public void setPower(double power) {
        shooterMotor.setPower(Math.max(-1.0, Math.min(1.0, power)));
    }

    /** Closed-loop velocity in revolutions per second. */
    public void setVelocity(double revsPerSecond) {
        shooterMotor.setVelocity(revsPerSecond * RobotConstants.SHOOTER_ENCODER_TICKS);
    }

    public void stop() {
        shooterMotor.setPower(0);
    }

    /** Current flywheel speed in revolutions per second. */
    public double getVelocity() {
        return shooterMotor.getVelocity() / RobotConstants.SHOOTER_ENCODER_TICKS;
    }

    public double getPower() {
        return shooterMotor.getPower();
    }

    /**
     * Keeps the wheel warm between shots instead of letting it spin all the way
     * down. Named for the speed rather than {@code idle()}, which the Subsystem
     * base class already uses for its do-nothing command factory.
     */
    public void holdIdleSpeed() {
        setVelocity(RobotConstants.IDLE_VELOCITY);
    }

    /** True once the wheel is at or just above target — safe to feed a ball. */
    public boolean atVelocity(double targetVelocity) {
        double actual = getVelocity();
        return actual > targetVelocity && actual < targetVelocity + velocityThreshold;
    }

    /** True once a shot has dragged the wheel down far enough to stop feeding. */
    public boolean belowVelocity(double targetVelocity) {
        return getVelocity() < targetVelocity - RobotConstants.VELOCITY_BOTTOM_THRESHOLD;
    }

    /** Live velocity PIDF, for the tuning opmodes. */
    public void setVelocityPIDF(double kp, double ki, double kd, double kf) {
        shooterMotor.setVelocityPIDFCoefficients(kp, ki, kd, kf);
    }

    public PIDFCoefficients getVelocityPIDF() {
        return shooterMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** Raw motor ticks per second, which is the unit the flywheel tuner works in. */
    public double getRawVelocity() {
        return shooterMotor.getVelocity();
    }

    /** Sets a raw ticks-per-second target, bypassing the rev/s conversion. */
    public void setRawVelocity(double ticksPerSecond) {
        shooterMotor.setVelocity(ticksPerSecond);
    }

    /* ===================== Shot solutions ===================== */

    /**
     * Flywheel velocity for a shot at {@code distanceInches}.
     *
     * <p>Returns the default velocity and leaves the remembered value alone when
     * the Limelight has nothing (distance is {@link RobotConstants#NO_TARGET}).
     */
    public double velocityForDistance(double distanceInches) {
        if (distanceInches == RobotConstants.NO_TARGET) {
            return RobotConstants.DEFAULT_VELOCITY;
        }
        lastKnownVelocity =
                RobotConstants.SHOT_SPEED_SLOPE * distanceInches + RobotConstants.SHOT_SPEED_INTERCEPT;
        return lastKnownVelocity;
    }

    /** Hood position for a shot at {@code distanceInches}. */
    public double hoodForDistance(double distanceInches) {
        if (distanceInches == RobotConstants.NO_TARGET) {
            return RobotConstants.HOOD_NEUTRAL;
        }
        lastKnownHood = RobotConstants.HOOD_SLOPE * distanceInches + RobotConstants.HOOD_INTERCEPT;
        return lastKnownHood;
    }

    /**
     * Open-loop power that holds {@code velocity} once the wheel is already there.
     *
     * <p>Handing over from the velocity PID to this keeps the wheel from dipping
     * when a ball passes through.
     */
    public double holdPowerFor(double velocity) {
        return RobotConstants.SHOT_POWER_SLOPE * velocity + RobotConstants.SHOT_POWER_INTERCEPT;
    }

    public double getLastKnownVelocity() {
        return lastKnownVelocity;
    }

    public double getLastKnownHood() {
        return lastKnownHood;
    }

    /* ===================== Hood ===================== */

    public void setHood(double position) {
        hoodServo.setPosition(position);
    }

    public void stowHood() {
        setHood(RobotConstants.HOOD_NEUTRAL);
    }

    /* ===================== Feed gate ===================== */

    /** Lets balls through to the flywheel. */
    public void openGate() {
        blockServo.setPosition(RobotConstants.BLOCK_OPEN);
        gateOpen = true;
    }

    /** Holds balls back from the flywheel. */
    public void closeGate() {
        blockServo.setPosition(RobotConstants.BLOCK_CLOSED);
        gateOpen = false;
    }

    public boolean isGateOpen() {
        return gateOpen;
    }

    /** Direct servo position, for the servo-test opmode. */
    public void setGatePosition(double position) {
        blockServo.setPosition(position);
    }
}
