package org.firstinspires.ftc.team2844.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vcs.valleylib.ftc.hardware.FtcSubsystem;

import org.firstinspires.ftc.team2844.helpers.Constants;

public class VisionSubsystem extends FtcSubsystem {
    private final Limelight3A limelight;
    private LLResult latestResult;
    private int pipeline;
    public VisionSubsystem(HardwareMap hardwareMap) {
        super(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, Constants.CHLL);
        limelight.start();
        latestResult = limelight.getLatestResult();
    }

    public void setPipeline(int pipeline){
        this.pipeline = pipeline;
        limelight.pipelineSwitch(pipeline);
    }

    public int getPipeline(){
        return pipeline;
    }

    @Override
    public void periodic() {
        latestResult = limelight.getLatestResult();
    }
}
