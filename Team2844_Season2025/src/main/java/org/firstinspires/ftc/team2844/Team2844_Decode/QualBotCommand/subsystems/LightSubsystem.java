package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.vcs.valleylib.core.time.RobotClock;
import com.vcs.valleylib.ftc.hardware.FtcSubsystem;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.RobotConstants;

/**
 * The goBILDA indicator light.
 *
 * <p>Once {@link #startMatchTimer()} is called it walks the light through the
 * match: one colour for the first minute, another through 1:40, a blink through
 * 2:00, then back to neutral. Driven from {@link #periodic()} so nothing has to
 * poll it.
 */
public class LightSubsystem extends FtcSubsystem {

    private final Servo light;

    private boolean timerRunning = false;
    private double matchStartSeconds = 0.0;
    private double lastBlinkSeconds = 0.0;
    private boolean blinkOn = false;

    public LightSubsystem(HardwareMap hardwareMap) {
        super(hardwareMap);
        light = hardwareMap.get(Servo.class, RobotConstants.LIGHT_SERVO);
        setColor(RobotConstants.LIGHT_NEUTRAL);
    }

    /** Raw colour, in the same 0-1 space as a servo position. */
    public void setColor(double color) {
        light.setPosition(color);
    }

    public void setNeutral() {
        setColor(RobotConstants.LIGHT_NEUTRAL);
    }

    /** Begins the match countdown display. Call when the opmode starts. */
    public void startMatchTimer() {
        matchStartSeconds = RobotClock.seconds();
        lastBlinkSeconds = matchStartSeconds;
        timerRunning = true;
    }

    @Override
    public void periodic() {
        if (!timerRunning) {
            return;
        }

        double now = RobotClock.seconds();
        double elapsed = now - matchStartSeconds;

        if (elapsed <= RobotConstants.EARLY_MATCH_SECONDS) {
            setColor(RobotConstants.LIGHT_EARLY_MATCH);
        } else if (elapsed <= RobotConstants.MID_MATCH_SECONDS) {
            setColor(RobotConstants.LIGHT_MID_MATCH);
        } else if (elapsed <= RobotConstants.END_MATCH_SECONDS) {
            if (now >= lastBlinkSeconds + RobotConstants.LIGHT_BLINK_SECONDS) {
                lastBlinkSeconds = now;
                blinkOn = !blinkOn;
                setColor(blinkOn ? RobotConstants.LIGHT_ENDGAME : RobotConstants.LIGHT_OFF);
            }
        } else {
            setColor(RobotConstants.LIGHT_NEUTRAL);
        }
    }
}
