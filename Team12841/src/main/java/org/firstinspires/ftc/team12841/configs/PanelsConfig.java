package org.firstinspires.ftc.team12841.configs;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class PanelsConfig {

    /* ===================== LIMELIGHT ===================== */

    public static double LLPGAIN = 0.023;
    public static final double LL_ALIGN_TOLERANCE = 0.4;

    /* ===================== SHOOTER PIDF ===================== */
    // REV velocity PIDF (RUN_USING_ENCODER)

    public static double SHOOTER_P = 0.0005;
    public static double SHOOTER_I = 0.0;
    public static double SHOOTER_D = 0.0;
    public static double SHOOTER_F = 0.056;

    /* ===================== SHOOTER TUNING ===================== */

    public static double SHOOTER_READY_RPM_ERROR = 75.0; // RPM window
    public static double REGRESSION_A = 1;
    public static double REGRESSION_B = 1;
    public static double REGRESSION_C = 1;

    /* ===================== SERVOS ===================== */

    public static double LEFT_SERVO_FLICK = 0;
    public static double RIGHT_SERVO_FLICK = 0;
    public static double LEFT_SERVO_IDLE = 0;
    public static double RIGHT_SERVO_IDLE = 0;
    public static double TT_POS0 = 0;
    public static double TT_POS1 = 0.5;
    public static double TT_POS2 = 1;
}
