package org.firstinspires.ftc.team2844.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vcs.valleylib.ftc.hardware.FtcSubsystem;

import org.firstinspires.ftc.team2844.helpers.Constants;

import java.util.concurrent.TimeUnit;

public class IntakeSubsystem extends FtcSubsystem {
    private final DcMotor intakeMotor;
    private final Servo flowerRamp;

    private final DigitalChannel[] nectar, pollen;
    private final ElapsedTime emptyFor = new ElapsedTime();
    boolean overflowed = false;
    int pollenCount, nectarCount;
    public IntakeSubsystem(HardwareMap hardwareMap) {
        super(hardwareMap);
        intakeMotor = hardwareMap.get(DcMotor.class, Constants.EHM0);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flowerRamp = hardwareMap.get(Servo.class, Constants.EHS0);


        final DigitalChannel n0, n1, n2, n3, p0, p1, p2, p3;
        n0 = hardwareMap.get(DigitalChannel.class, "n0");
        n1 = hardwareMap.get(DigitalChannel.class, "n1");
        n2 = hardwareMap.get(DigitalChannel.class, "n2");
        n3 = hardwareMap.get(DigitalChannel.class, "n3");
        nectar = new DigitalChannel[]{n0, n1, n2, n3};

        p0 = hardwareMap.get(DigitalChannel.class, "p0");
        p1 = hardwareMap.get(DigitalChannel.class, "p1");
        p2 = hardwareMap.get(DigitalChannel.class, "p2");
        p3 = hardwareMap.get(DigitalChannel.class, "p3");
        pollen = new DigitalChannel[]{p0, p1, p2, p3};
    }

    /**
     * Run the Intake
     * */
    public void intake(){
        intakeMotor.setPower(Constants.INTAKE_SPEED);
    }

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

    public void flowerRampDown(){
        flowerRamp.setPosition(Constants.FLOWER_DOWN);
    }

    public void flowerRampUp(){
        flowerRamp.setPosition(Constants.FLOWER_UP);
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

    public int nectarCount(){
        return nectarCount;
    }

    public int pollenCount(){
        return pollenCount;
    }

    /**
     * Total limit switches tripped inside the robot
     * */
    public int ballCount(){
        return nectarCount() + pollenCount();
    }

    @Override
    public void periodic() {
        //if there are too many balls in the robot, spit them out. Once there are no longer too many, stop outtaking
        if(overfull() && !intakeMotor.isBusy()){
            outtake();
            overflowed = true;
        } else if(overflowed){
            stopIntake();
            overflowed = false;
        }

        //counts the nectar switches once per loop instead of every time the method is called
        nectarCount = 0;
        for(DigitalChannel n: nectar){ //for each limit switch
            if(n.getState()){
                nectarCount++; //count how many are tripped
            }
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
