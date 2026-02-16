package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;

public class ShooterFeedSubsystem extends SubsystemBase {
    /**The continuous servo that runs the feeder in the neck of the shooter*/
    private Motor tFeed;
    private DigitalChannel topBB;

    public ShooterFeedSubsystem(Motor tFeed, DigitalChannel topBB){
        this.tFeed = tFeed;
        this.topBB = topBB;
    }

    public void runTFeedForward() {
        tFeed.set(1.0);
    }

    public void stopTFeed(){
        tFeed.set(0.0);
    }

    public void slowFeed(){
        tFeed.set(Constants.SLOW_TFEED);
    }

    public void runTFeedBackward(){tFeed.set(-1.0);}

    /** returns true if there is a ball in the top chamber*/
    public boolean topBroken(){return topBB.getState();}
}
