package org.firstinspires.ftc.team12841.configs;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class PanelsConfig {

    /* ===================== LIMELIGHT ===================== */

    public static double LLPGAIN = 0.06;
    public static final double LL_ALIGN_TOLERANCE = 0.4;

    /* ===================== SHOOTER TUNING ===================== */

    public static double REGRESSION_A = -0.00000966113;
    public static double REGRESSION_B = 0.00533861;
    public static double REGRESSION_C = 0.837588;
    public static double REGRESSION_D = 54.5793;
    public static double REGRESSION_F = 1458.64071;

    // y=-0.00000966113x^{4}+0.00533861x^{3}-0.837588x^{2}+54.5793x+1458.64071

    public static double BABY = 0.4;

}
