package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SortingObjects;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Constants;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class ExpectedSpindexer {
    public Artifact[] simSpindexer = new Artifact[3];
    public Artifact[] realSpindexer = new Artifact[3];

    public ExpectedSpindexer(){
        simSpindexer[0] = new Artifact(Constants.UNKNOWN_COLOR, 1);
        simSpindexer[1] = new Artifact(Constants.UNKNOWN_COLOR, 2);
        simSpindexer[2] = new Artifact(Constants.UNKNOWN_COLOR, 3);
    }
}
