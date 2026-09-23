package org.firstinspires.ftc.team12841.configs;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class PanelsConfig {

    /* ===================== LIMELIGHT ===================== */

    public static double LLPGAIN = 0.04;
    public static final double LL_ALIGN_TOLERANCE = 0.4;

    /* ===================== SHOOTER STUFF ===================== */

    public static double OUT_POS = 0.5;
    public static double IN_POS = 0.0;

    public static double ENCODER_TICS = 28.0; // Acts as SHOOTER_TICKS_PER_REV
    public static double velThresh = 100;
    public static double VEL_BOTTOM_THRESH = 1.0;

    public static double hoodAim = 0;

    public static double ghettoIn = 0;
    public static double ghettoOut = 0.2;
    public static double BABY = 0.4;

}
