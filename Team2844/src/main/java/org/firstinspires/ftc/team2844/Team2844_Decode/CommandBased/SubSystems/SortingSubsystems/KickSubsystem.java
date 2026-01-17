package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


public class KickSubsystem extends SubsystemBase {
    /* ---------- Declarations ----------*/
    /**The servo that rotates the kicker into place*/
    private Servo kickerRotate;
    /**kickerSpin spins the end of the kicker*/
    private CRServo kickerSpin;
    /**sFeed spins the wheel on the opposite side of the kicker.*/
    private CRServo sFeed;

    /**
     * The constructor for the KickSubsystem, it sets the motors/servos equal to the object passed in,
     * do so by calling:
     * new Motor(hardwaremap, "____"); for motors,
     * and hardwaremap.get(___.class, "____"); for servos
     * @param kickerRotate The servo that rotates the kicker into place, requires passing in new Servo object
     * @param kickerSpin kickerSpin spins the end of the kicker, requires passing in new CRServo object
     * @param sFeed sFeed spins the wheel on the opposite side of the kicker, requires passing in new CRServo object
     */
    public KickSubsystem(Servo kickerRotate, CRServo kickerSpin, CRServo sFeed){
        this.kickerRotate = kickerRotate;
        this.kickerSpin = kickerSpin;
        this.sFeed = sFeed;

    }
}
