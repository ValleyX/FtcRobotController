package org.firstinspires.ftc.team2844.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.vcs.valleylib.ftc.hardware.FtcSubsystem;
import org.firstinspires.ftc.team2844.helpers.Constants;

public class TransferSubsystem extends FtcSubsystem {
    private final DcMotor transferMotor;
    private final Servo nectarGate, pollenGate;
    private int ballCount;
    private boolean overflowed;
    public TransferSubsystem(HardwareMap hardwareMap) {
        super(hardwareMap);
        transferMotor = hardwareMap.get(DcMotor.class, Constants.EHM1);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        nectarGate = hardwareMap.get(Servo.class, Constants.CHS2);
        pollenGate = hardwareMap.get(Servo.class, Constants.CHS3);
    }

    public void transfer(){
        transferMotor.setPower(Constants.TRANSFER_SPEED);
    }

    public void stop(){
        transferMotor.setPower(0.0);
    }

    public void spitTransfer(){
        transferMotor.setPower(-Constants.TRANSFER_SPEED);
    }

    public boolean stopIfFull(){
        if(ballCount == 4){
            stop();
            return true;
        }
        return false;
    }

    public boolean full(){
        return ballCount == 4;
    }

    public void openNectarGate(){
        nectarGate.setPosition(Constants.OPEN_GATE);
    }
    public void closeNectarGate(){
        nectarGate.setPosition(Constants.CLOSED_GATE);
    }

    public void openPollenGate(){
        pollenGate.setPosition(Constants.OPEN_GATE);
    }
    public void closePollenGate(){
        pollenGate.setPosition(Constants.CLOSED_GATE);
    }

    public void openGates(){
        openNectarGate();
        openPollenGate();
    }

    public void closeGates(){
        closeNectarGate();
        closePollenGate();
    }

    public void setBallCount(int ballCount){
        this.ballCount = ballCount;
    }

    @Override
    public void periodic() {
        if(ballCount > 4 && transferMotor.getPower() != 0.0){
            spitTransfer();
            overflowed = true;
        } else if(overflowed && !(ballCount > 4)){
            stop();
            overflowed = false;
        }
    }
}
