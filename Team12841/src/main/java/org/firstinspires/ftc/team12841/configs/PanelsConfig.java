package org.firstinspires.ftc.team12841.configs;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class PanelsConfig {

    /* ===================== LIMELIGHT ===================== */

    public static double LLPGAIN = 0.005;
    public static final double LL_ALIGN_TOLERANCE = 0.4;

    /* ===================== SHOOTER TUNING ===================== */

    public static double SHOOTER_READY_RPM_ERROR = 75.0; // RPM window
    public static double REGRESSION_A = 0.3334689294;
    public static double REGRESSION_B = -51.09719017;
    public static double REGRESSION_C = 4694.418688;

    /* ===================== SERVOS ===================== */

    public static double LEFT_SERVO_FLICK = .19;
    public static double RIGHT_SERVO_FLICK = 0;
    public static double LEFT_SERVO_IDLE = 0;
    public static double LEFT_SERVO_READY = 0.23;
    public static double RIGHT_SERVO_IDLE = 0.5;
    public static double TT_POS0 = 0;
    public static double TT_POS1 = 0.44;
    public static double TT_POS2 = 0.88;
    public static double TT_POS3 = 1;

}
