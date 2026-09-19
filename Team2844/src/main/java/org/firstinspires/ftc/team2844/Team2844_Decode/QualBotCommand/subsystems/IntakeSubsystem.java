package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.vcs.valleylib.ftc.hardware.FtcSubsystem;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.RobotConstants;

/**
 * Intake roller, the beam breaks that count balls, and the "ghetto arm" ball stop.
 *
 * <p>The roller doubles as the feeder — running it forward with the shooter's
 * gate open is what pushes balls into the flywheel — so shooting commands
 * require this subsystem as well as the shooter.
 */
public class IntakeSubsystem extends FtcSubsystem {

    private final DcMotor intakeMotor;
    private final Servo ballStop;

    private final DigitalChannel beamBreak0;
    private final DigitalChannel beamBreak1;
    private final DigitalChannel beamBreak2;
    private final DigitalChannel beamBreak3;
    private final DigitalChannel gobildaBeamBreak;
    private final DigitalChannel gobildaBeamBreak1;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        super(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotor.class, RobotConstants.INTAKE_MOTOR);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ballStop = hardwareMap.get(Servo.class, RobotConstants.BALL_STOP_SERVO);

        beamBreak0 = hardwareMap.get(DigitalChannel.class, RobotConstants.BEAM_BREAK_0);
        beamBreak1 = hardwareMap.get(DigitalChannel.class, RobotConstants.BEAM_BREAK_1);
        beamBreak2 = hardwareMap.get(DigitalChannel.class, RobotConstants.BEAM_BREAK_2);
        beamBreak3 = hardwareMap.get(DigitalChannel.class, RobotConstants.BEAM_BREAK_3);
        gobildaBeamBreak = hardwareMap.get(DigitalChannel.class, RobotConstants.GOBILDA_BEAM_BREAK);
        gobildaBeamBreak1 = hardwareMap.get(DigitalChannel.class, RobotConstants.GOBILDA_BEAM_BREAK_1);
    }

    /* ===================== Roller ===================== */

    /** Runs the roller, clamped to [-1, 1]. Negative spits balls back out. */
    public void setPower(double power) {
        intakeMotor.setPower(Math.max(-1.0, Math.min(1.0, power)));
    }

    public void intake(double power) {
        setPower(Math.abs(power));
    }

    public void extake(double power) {
        setPower(-Math.abs(power));
    }

    public void stop() {
        setPower(0.0);
    }

    public double getPower() {
        return intakeMotor.getPower();
    }

    /* ===================== Ball stop ("ghetto arm") ===================== */

    /** Drops the arm onto the ball so it can't roll away. */
    public void holdBall() {
        ballStop.setPosition(RobotConstants.BALL_STOP_HOLD);
    }

    /** Lifts the arm clear so the ball can be fed to the shooter. */
    public void releaseBall() {
        ballStop.setPosition(RobotConstants.BALL_STOP_RELEASE);
    }

    /* ===================== Ball counting ===================== */

    /**
     * The three storage slots, read off the beam breaks.
     *
     * <p>Slots 1 and 2 have a redundant pair of breaks each and read
     * active-low; the goBILDA breaks read active-high. Exactly as the old
     * {@code ShooterHardware} had it.
     */
    private boolean slot1() {
        return (!beamBreak0.getState() || !beamBreak1.getState()) && gobildaBeamBreak1.getState();
    }

    private boolean slot2() {
        return !beamBreak2.getState() || !beamBreak3.getState();
    }

    private boolean slot3() {
        return gobildaBeamBreak.getState();
    }

    /** At least one ball on board. */
    public boolean hasBall() {
        return slot1() || slot2() || slot3();
    }

    /** At least two balls on board. */
    public boolean hasTwoBalls() {
        return (slot1() && slot2()) || (slot1() && slot3()) || (slot2() && slot3());
    }

    /** Full — all three slots occupied. */
    public boolean isFull() {
        return slot1() && slot2() && slot3();
    }
}
