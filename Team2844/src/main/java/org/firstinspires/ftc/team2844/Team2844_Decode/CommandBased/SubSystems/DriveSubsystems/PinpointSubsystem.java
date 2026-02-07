package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.RoadrunnerQuickstart.PinpointLocalizer;

public class PinpointSubsystem extends SubsystemBase {
    private GoBildaPinpointDriver pinpoint;

    public PinpointSubsystem(GoBildaPinpointDriver pinpoint){
        this.pinpoint = pinpoint;
    }
}
