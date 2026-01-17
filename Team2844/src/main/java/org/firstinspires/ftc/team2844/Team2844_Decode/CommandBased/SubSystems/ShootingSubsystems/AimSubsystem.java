package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class AimSubsystem extends SubsystemBase {

    //   SERVO DECLARATIONS
    /**The servo that controls the angle the artifact is shot at*/
    private Servo hoodAim;
    /**The servo that controls the heading of the turret.*/
    private Servo turretAim;

    /**
     * The constructor for the AimSubsystem, it sets the motors/servos equal to the object passed in,
     * do so by calling:
     * new Motor(hardwaremap, "____"); for motors,
     * and hardwaremap.get(___.class, "____"); for servos
     * @param hoodAim The servo that controls the angle the artifact is shot at
     * @param turretAim The servo that controls the heading of the turret.
     */
    public AimSubsystem(Servo hoodAim, Servo turretAim){
        this.hoodAim = hoodAim;
        this.turretAim = turretAim;
    }
}
