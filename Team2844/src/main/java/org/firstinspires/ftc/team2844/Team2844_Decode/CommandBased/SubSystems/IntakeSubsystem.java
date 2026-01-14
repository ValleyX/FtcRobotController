package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

/*
This Class will run the Intake system motor
 */
public class IntakeSubsystem extends SubsystemBase {

    private Motor IntakeMotor;

    /*
    IntakeSubsystem Class Constructor
     */
    public IntakeSubsystem(Motor IntakeMotor) {
       this.IntakeMotor = IntakeMotor;
    }

    public void activate() {
        IntakeMotor.set(0.75);
       // rightMotor.set(-0.75);
    }

    public void stop() {
        IntakeMotor.set(0);
        //rightMotor.set(0);
    }

    public void reverse() {
        IntakeMotor.set(-0.75);
        //rightMotor.set(0.75);
    }


}
