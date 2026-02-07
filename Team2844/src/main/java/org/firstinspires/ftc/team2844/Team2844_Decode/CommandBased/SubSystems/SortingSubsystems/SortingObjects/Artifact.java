package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SortingObjects;

public class Artifact {
    int color;
    int pos;
    public Artifact(int color, int pos){
        this.color = color;
        this.pos = pos;
    }

    public int getColor(){
        return color;
    }

    public int getPos(){
        return pos;
    }

    public void setPos(int pos){
        this.pos = pos;
    }
}
