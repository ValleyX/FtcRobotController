package org.firstinspires.ftc.team12841.configs;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class PanelsConfig {

    /* ===================== LIMELIGHT ===================== */

    public static double LLPGAIN = 0.023;

    /* ===================== SHOOTER PIDF ===================== */
    // REV velocity PIDF (RUN_USING_ENCODER)

    public static double SHOOTER_P = 0.0005;
    public static double SHOOTER_I = 0.0;
    public static double SHOOTER_D = 0.0;
    public static double SHOOTER_F = 0.056;

    /* ===================== SHOOTER TUNING ===================== */

    public static double SHOOTER_READY_RPM_ERROR = 75.0; // RPM window
}
