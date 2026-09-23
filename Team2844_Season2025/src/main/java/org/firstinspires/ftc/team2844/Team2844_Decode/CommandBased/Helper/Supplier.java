package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper;

import java.util.function.DoubleSupplier;

public class Supplier implements DoubleSupplier {
    double num;

    public Supplier(){
        num = 0;
    }

    public Supplier(double num){
        this.num = num;
    }

    public void setNum(double num){
        this.num = num;
    }

    public void addToNum(double num){
        this.num += num;
    }

    public void subtractNum(double num){
        this.num -= num;
    }

    @Override
    public double getAsDouble() {
        return num;
    }
}
