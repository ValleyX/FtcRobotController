package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class FallingEdgeTrigger {
    private Supplier<Boolean> booleanSupplier;
    private boolean previousState = true;

    public FallingEdgeTrigger(Supplier<Boolean> booleanSupplier){
        this.booleanSupplier = booleanSupplier;
    }

    public boolean get(){
        boolean current = booleanSupplier.get();
        boolean trigger = !current && previousState;
        previousState = current;
        return trigger;
    }

    public BooleanSupplier asSupplier(){
        return this::get;
    }
}
