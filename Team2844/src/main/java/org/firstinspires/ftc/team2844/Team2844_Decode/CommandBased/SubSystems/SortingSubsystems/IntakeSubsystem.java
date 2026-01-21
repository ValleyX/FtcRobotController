package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

/*
This Class will run the Intake system motor
 */
public class IntakeSubsystem extends SubsystemBase {

    private Motor intakeMotor;

    /*
    IntakeSubsystem Class Constructor
     */
    public IntakeSubsystem(Motor intakeMotor) {
       this.intakeMotor = intakeMotor;
    }

    public void activate(double power) {
        intakeMotor.set(power);
       // rightMotor.set(-0.75);
    }

    public void stop() {
        intakeMotor.set(0);
        //rightMotor.set(0);
    }

    public void reverse() {
        intakeMotor.set(-0.75);
        //rightMotor.set(0.75);
    }


}
