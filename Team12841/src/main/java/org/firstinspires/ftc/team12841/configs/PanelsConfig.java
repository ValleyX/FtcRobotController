package org.firstinspires.ftc.team12841.configs;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class PanelsConfig {

    /* ===================== LIMELIGHT ===================== */

    public static double LLPGAIN = 0.04;
    public static final double LL_ALIGN_TOLERANCE = 0.4;

    /* ===================== SHOOTER TUNING ===================== */

    public static double REGRESSION_A = 0.0980506327;
    public static double REGRESSION_B = -9.573856909;
    public static double REGRESSION_C = 2931.96567;

    // y=0.0980506327x^{2}-9.573856909x+2931.96567

    public static double BABY = 0.4;

}
