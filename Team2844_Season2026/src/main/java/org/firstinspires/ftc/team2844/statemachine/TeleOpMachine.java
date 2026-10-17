package org.firstinspires.ftc.team2844.statemachine;

import com.vcs.valleylib.core.fsm.StateMachine;

import org.firstinspires.ftc.team2844.commands.CommandFactory;

public class TeleOpMachine {
    public enum Cycle {INTAKE, SHOOT};

    StateMachine<Cycle> teleOpCycle = new StateMachine<>(Cycle.INTAKE)
            .state(Cycle.INTAKE, CommandFactory.IntakeState())
            .state(Cycle.SHOOT, CommandFactory.ShootState())
            .transitionOnFinish(Cycle.INTAKE, Cycle.SHOOT)
            .transitionOnFinish(Cycle.SHOOT, Cycle.INTAKE);

    //TODO add real commands for states, and transition when full and empty.
    //TODO add shooting state that turns off intake and transfer until velocity is in range and opens gates

}
