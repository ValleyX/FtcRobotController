package org.firstinspires.ftc.team2844simple.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.vcs.valleylib.ftc.hardware.FtcSubsystem;
import org.firstinspires.ftc.team2844simple.helpers.Constants;

public class TransferSubsystem extends FtcSubsystem {
    private final DcMotor transferMotor;
    private final Servo gate1, gate2;
    //private final Servo gate;
    private int ballCount;
    private boolean overflowed;
    private boolean pollenGateOpen;
    public TransferSubsystem(HardwareMap hardwareMap) {
        super(hardwareMap);
        transferMotor = hardwareMap.get(DcMotor.class, Constants.EHM1);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        gate1 = hardwareMap.get(Servo.class, Constants.CHS0);
        gate2 = hardwareMap.get(Servo.class, Constants.EHS0);
        gate1.setDirection(Servo.Direction.REVERSE);
        gate2.setDirection(Servo.Direction.FORWARD);
        pollenGateOpen = false;
//        gate = hardwareMap.get(Servo.class, Constants.CHS2);
    }

    public void transfer(){
        transferMotor.setPower(Constants.TRANSFER_SPEED);
    }
    public void fastTransfer(){transferMotor.setPower(Constants.FAST_TRANSFER);}
    public void transfer(double speed){transferMotor.setPower(speed);}

    public void stop(){
        transferMotor.setPower(0.0);
    }

    public void spitTransfer(){
        transferMotor.setPower(-Constants.TRANSFER_SPEED);
    }


    public void openTransferGates(){
        gate1.setPosition(Constants.OPEN_GATE);
        gate2.setPosition(Constants.OPEN_GATE);
        pollenGateOpen = true;
    }
    public void closeTransferGates(){
        gate1.setPosition(Constants.CLOSED_GATE);
        gate2.setPosition(Constants.CLOSED_GATE);
        pollenGateOpen = false;
    }

    public boolean isPollenGateOpen(){
        return pollenGateOpen;
    }

    public boolean areGatesOpen(){
        return isPollenGateOpen();
    }

    public void openGates(){
        //gate.setPosition(Constants.CLOSED_GATE);
        openTransferGates();
    }

    public void closeGates(){
        //gate.setPosition(Constants.OPEN_GATE);
        closeTransferGates();
    }

    public double getGate1Pos(){
        return gate1.getPosition();
    }

    public double getGate2Pos(){
        return gate2.getPosition();
    }

    @Override
    public void periodic() {
    }
}
