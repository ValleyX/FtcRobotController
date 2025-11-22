package org.firstinspires.ftc.team12841.configs;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class TeleOpConfig {
    public static double SHOOTER_IDLE  = 0.67;
    public static double SHOOTER_FIRE  = 0.48;
    public static double TT_POS_0      = 0.01;
    public static double TT_POS_1      = 0.51;
    public static double TT_POS_2      = 0.97;
    public static double BABY_MODE_SCALE = 0.2;
    public static double OLDMAN_MODE_SCALE = 0.4;
    public static double KP = 0.02;
    public static double TURN_THRESH = 2.3;   // allowable yaw error
    public static double SLOW_THRESH = 15;    // unused here but used for slow mode
    public static double PGAIN = 0.023;       // proportional gain for IMU-based turning
}
