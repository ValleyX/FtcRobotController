package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand;

/**
 * Every tuned number for the Qual Bot, in one place.
 *
 * <p>All of these were lifted straight out of the old {@code QualBot} package
 * (RobotHardware / ShooterHardware / OnePersonDriveBase) so the command-based
 * robot behaves identically to the LinearOpMode version it replaces.
 */
public final class RobotConstants {

    private RobotConstants() {}

    /* ===================== Hardware map names ===================== */

    public static final String LEFT_FRONT_MOTOR = "leftFront";
    public static final String LEFT_BACK_MOTOR = "leftBack";
    public static final String RIGHT_FRONT_MOTOR = "rightFront";
    public static final String RIGHT_BACK_MOTOR = "rightBack";

    public static final String IMU = "imu";
    public static final String LIMELIGHT = "limelight";
    public static final String CONTROL_HUB = "Control Hub";

    public static final String SHOOTER_MOTOR = "shooter";
    public static final String INTAKE_MOTOR = "intake";

    public static final String BLOCK_SERVO = "blockSer";
    public static final String HOOD_SERVO = "hoodSer";
    public static final String BALL_STOP_SERVO = "ballstop";
    public static final String LIGHT_SERVO = "gobildaLight";

    public static final String BEAM_BREAK_0 = "BB0";
    public static final String BEAM_BREAK_1 = "BB1";
    public static final String BEAM_BREAK_2 = "BB2";
    public static final String BEAM_BREAK_3 = "BB3";
    public static final String GOBILDA_BEAM_BREAK = "gobuildaBB";
    public static final String GOBILDA_BEAM_BREAK_1 = "gobuildaBB1";

    /* ===================== Drive ===================== */

    /** Scales the turn stick. */
    public static final double ROT_CORRECTION = 1.0;

    /** "Baby mode" precision-driving multiplier (right stick button toggles it). */
    public static final double BABY_MODE_MULTIPLIER = 0.4;

    /** Deadband applied to the driver sticks before they reach the follower. */
    public static final double STICK_DEADBAND = 0.05;

    /* ===================== Limelight aiming ===================== */

    /** Sentinel the Limelight wrapper returns when it has no valid target. */
    public static final double NO_TARGET = -999;

    /**
     * Turn power per degree of Limelight tx error.
     *
     * <p>The old {@code turnToFree()} used {@code speed(0.6) * PGAIN(0.04) * errorDegrees},
     * which works out to this same 0.024 power/degree.
     */
    public static final double AIM_KP = 0.024;

    /** Don't bother correcting inside this many degrees (old {@code TURN_THRESH}). */
    public static final double AIM_TOLERANCE_DEGREES = 1.8;

    /** Cap on the aim-assist turn output (old {@code speed} argument). */
    public static final double AIM_MAX_POWER = 0.6;

    /** Fallback shot distance (inches) when the Limelight can't see a tag in auto. */
    public static final double FALLBACK_SHOT_DISTANCE = 120.0;

    /* ===================== Shooter ===================== */

    /** Ticks per revolution of the bare shooter motor. */
    public static final double SHOOTER_ENCODER_TICKS = 28;

    /** Velocity PIDF for the shooter's RUN_USING_ENCODER mode. */
    public static final double SHOOTER_P = 60.0;
    public static final double SHOOTER_I = 0.0;
    public static final double SHOOTER_D = 10.0;
    public static final double SHOOTER_F = 35.0;

    /** How far above target still counts as "at speed" in teleop. */
    public static final double VELOCITY_THRESHOLD_TELEOP = 10.0;

    /** Tighter window used in auto, where there is time to wait for a real spin-up. */
    public static final double VELOCITY_THRESHOLD_AUTO = 1.0;

    /** How far below target forces the feed to stop and let the wheel recover. */
    public static final double VELOCITY_BOTTOM_THRESHOLD = 1.5;

    /** Speed the shooter idles at between shots so it doesn't spin down completely. */
    public static final double IDLE_VELOCITY = 20.0;

    /** Used when there is no distance reading at all. */
    public static final double DEFAULT_VELOCITY = 30.0;

    /** Teleop dpad trim step for the target velocity. */
    public static final double VELOCITY_TRIM_STEP = 0.5;

    /** shooterVelocity = SHOT_SPEED_SLOPE * distanceInches + SHOT_SPEED_INTERCEPT (JAE 2-19-26). */
    public static final double SHOT_SPEED_SLOPE = 0.1329324;
    public static final double SHOT_SPEED_INTERCEPT = 29.47163;

    /** hoodPosition = HOOD_SLOPE * distanceInches + HOOD_INTERCEPT. */
    public static final double HOOD_SLOPE = 0.0038895;
    public static final double HOOD_INTERCEPT = -0.111817;

    /** Open-loop power that holds a given velocity once the wheel is already there. */
    public static final double SHOT_POWER_SLOPE = 0.012034;
    public static final double SHOT_POWER_INTERCEPT = -0.00615;

    /** Hood position for "stowed". */
    public static final double HOOD_NEUTRAL = 0.0;

    /** Shooter power used in auto to hold the wheel warm while driving to a shot. */
    public static final double AUTO_WARMUP_POWER = 0.2;

    /** Reverse shooter power that keeps balls from feeding through while intaking. */
    public static final double AUTO_INTAKE_REVERSE_POWER_HIGH_VOLTAGE = -0.35;
    public static final double AUTO_INTAKE_REVERSE_POWER_LOW_VOLTAGE = -0.25;

    /** Intake power while collecting in auto, by battery state. */
    public static final double AUTO_INTAKE_POWER_HIGH_VOLTAGE = 0.9;
    public static final double AUTO_INTAKE_POWER_LOW_VOLTAGE = 1.0;

    /** Battery voltage above which the robot is treated as "fresh". */
    public static final double HIGH_VOLTAGE_THRESHOLD = 13.0;

    /** Seconds of extra feeding after the last ball clears the sensors. */
    public static final double SHOOT_BUFFER_SECONDS = 1.2;

    /* ===================== Servos ===================== */

    /** Block servo position that lets balls through to the shooter (old {@code IN_POS}). */
    public static final double BLOCK_OPEN = 0.0;

    /** Block servo position that holds balls back (old {@code OUT_POS}). */
    public static final double BLOCK_CLOSED = 0.2;

    /** "Ghetto arm" down, holding a ball in place. */
    public static final double BALL_STOP_HOLD = 1.0;

    /** "Ghetto arm" lifted out of the way so the ball can be shot. */
    public static final double BALL_STOP_RELEASE = 0.35;

    /* ===================== Indicator light ===================== */

    public static final double LIGHT_OFF = 0.0;
    public static final double LIGHT_NEUTRAL = 0.611;
    public static final double LIGHT_EARLY_MATCH = 0.5;
    public static final double LIGHT_MID_MATCH = 0.333;
    public static final double LIGHT_ENDGAME = 0.28;

    /** Seconds between blinks during endgame. */
    public static final double LIGHT_BLINK_SECONDS = 0.2;

    public static final double EARLY_MATCH_SECONDS = 60;
    public static final double MID_MATCH_SECONDS = 100;
    public static final double END_MATCH_SECONDS = 120;
}
