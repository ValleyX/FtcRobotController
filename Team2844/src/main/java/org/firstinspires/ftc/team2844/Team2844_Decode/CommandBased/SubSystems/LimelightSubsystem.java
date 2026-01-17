package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class LimelightSubsystem extends SubsystemBase {

    /* ----------- Declarations ----------- */
    /**The man, the myth, the legend, the limelight3A*/
    private Limelight3A limelight;

    /**
     * This is the limelight Constructor
     * also requires the pipeline number to be passed in,
     * 0 is blue,
     * 1 is red
     * @param limelight Requires passing in a limelight object to set the limelight to
     * @param pipelineNumber The number of pipeline to use (0 is blue, 1 is red)
     */
    public LimelightSubsystem(Limelight3A limelight, int pipelineNumber){
        //initialize the limelight
        this.limelight = limelight;

        //switches the pipeline
        limelight.pipelineSwitch(pipelineNumber);
        limelight.start();
    }
}
