package org.firstinspires.ftc.team2844simple.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vcs.valleylib.ftc.hardware.FtcSubsystem;

import org.firstinspires.ftc.team2844simple.helpers.Constants;

import java.util.concurrent.TimeUnit;

public class IntakeSubsystem extends FtcSubsystem {
    private final DcMotor intakeMotor;

    private final DigitalChannel[] pollen;
    private final ElapsedTime emptyFor = new ElapsedTime();
    boolean overflowed = false;
    int pollenCount;
    public IntakeSubsystem(HardwareMap hardwareMap) {
        super(hardwareMap);
        intakeMotor = hardwareMap.get(DcMotor.class, Constants.EHM0);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        final DigitalChannel p0, p1, p2, p3;
        p0 = hardwareMap.get(DigitalChannel.class, Constants.EHDD4);
        p1 = hardwareMap.get(DigitalChannel.class, Constants.EHDD5);
        p2 = hardwareMap.get(DigitalChannel.class, Constants.EHDD6);
        p3 = hardwareMap.get(DigitalChannel.class, Constants.EHDD7);
        pollen = new DigitalChannel[]{p0, p1, p2};
        for (DigitalChannel p : pollen){
            p.setMode(DigitalChannel.Mode.INPUT);
        }
    }

    /**
     * Run the Intake
     * */
    public void intake(){
        intakeMotor.setPower(Constants.INTAKE_SPEED);
    }
    public void intake(double speed){intakeMotor.setPower(speed);}

    /**
     * Stop the intake
     * */
    public void stopIntake(){
        intakeMotor.setPower(0.0);
    }

    /**
     * Run the intake backwards
     * */
    public void outtake(){
        intakeMotor.setPower(-Constants.INTAKE_SPEED);
    }

    /**
     * Stop the intake if four limit switches are tripped
     * */
    public boolean stopIfFull(){
        if(full()){
            stopIntake();
            return true;
        }
        return false;
    }

    public boolean full(){
        return ballCount() == 4;
    }

    public boolean empty(){
        return emptyFor.time(TimeUnit.MILLISECONDS) > Constants.EMPTY_TIME;
    }

    /**
     * More than four total limit switches are tripped
     * */
    public boolean overfull(){
        return ballCount() > 4;
    }

    public int pollenCount(){
        return pollenCount;
    }

    /**
     * Total limit switches tripped inside the robot
     * */
    public int ballCount(){
        return pollenCount();
    }

    @Override
    public void periodic() {
        //if there are too many balls in the robot, spit them out. Once there are no longer too many, stop outtaking
        if(overfull() && (intakeMotor.getPower() == 0.0)){
            outtake();
            overflowed = true;
        } else if(overflowed && !overfull()){
            stopIntake();
            overflowed = false;
        }

        //counts the pollen switches once per loop instead of every time the method is called
        pollenCount = 0;
        for(DigitalChannel p: pollen){
            if(p.getState()){
                pollenCount++;
            }
        }

        if(ballCount() != 0){
            emptyFor.reset();
        }
    }
}
