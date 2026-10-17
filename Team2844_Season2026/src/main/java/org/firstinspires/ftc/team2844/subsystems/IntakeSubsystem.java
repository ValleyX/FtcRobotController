package org.firstinspires.ftc.team2844.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vcs.valleylib.ftc.hardware.FtcSubsystem;

import org.firstinspires.ftc.team2844.helpers.Constants;

public class IntakeSubsystem extends FtcSubsystem {
    private final DcMotor intakeMotor, transferMotor;
    private final DigitalChannel n0, n1, n2, n3, p0, p1, p2, p3;
    private DigitalChannel[] nectar, pollen;
    public IntakeSubsystem(HardwareMap hardwareMap) {
        super(hardwareMap);
        intakeMotor = hardwareMap.get(DcMotor.class, Constants.EHM0);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transferMotor = hardwareMap.get(DcMotor.class, Constants.EHM1);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    public void intake(){
        intakeMotor.setPower(Constants.INTAKE_SPEED);
    }

    public void stopIntake(){
        intakeMotor.setPower(0.0);
    }

    public void outtake(){
        intakeMotor.setPower(-Constants.INTAKE_SPEED);
    }

    public void transfer(){
        transferMotor.setPower(Constants.TRANSFER_SPEED);
    }

    public void stopTransfer(){
        transferMotor.setPower(0.0);
    }

    public void spitTransfer(){
        transferMotor.setPower(-Constants.TRANSFER_SPEED);
    }

    public void stop(){
        stopIntake();
        stopTransfer();
    }

    public void spit(){
        outtake();
        spitTransfer();
    }

    public boolean stopIfFull(){
        if(full()){
            stopIntake();
            return true;
        }
        return false;
    }

    public boolean full(){
        return (nectarCount() + pollenCount()) == 4;
    }

    public boolean empty(){
        return (nectarCount() + pollenCount()) == 0;
    }

    public boolean overfull(){
        return (nectarCount()+pollenCount()) > 4;
    }

    public int nectarCount(){
        int i = 0;
        for(DigitalChannel n: nectar){
            if(n.getState()){
                i++;
            }
        }
        return i;
    }

    public int pollenCount(){
        int i = 0;
        for(DigitalChannel n: pollen){
            if(n.getState()){
                i++;
            }
        }
        return i;
    }

    @Override
    public void periodic() {
        if(overfull()){
            spit();
        }
    }
}
