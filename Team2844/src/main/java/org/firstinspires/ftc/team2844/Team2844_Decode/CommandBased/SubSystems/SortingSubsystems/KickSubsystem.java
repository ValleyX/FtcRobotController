package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Constants;


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

    public void runSFeedForward(){
        sFeed.setPower(-1);
    }

    public void runSFeedBackward(){
        sFeed.setPower(1);
    }

    public void stopSFeed(){
        sFeed.setPower(0.0);
    }

    public void runKickerSpin(){
        kickerSpin.setPower(-1);
    }

    public void runKickerSpinBackwards(){
        kickerSpin.setPower(1);
    }

    public void stopKickerSpin(){
        kickerSpin.setPower(0.0);
    }

    public void rotateKickerDown(){
        kickerRotate.setPosition(Constants.MAX_KICKDOWN);
    }

    public void rotateKickerUp(){
        kickerRotate.setPosition(0.0);
    }

    public double getKickerRotate(){
        return kickerRotate.getPosition();
    }
}
