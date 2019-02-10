package frc.robot.efrat.statemachine;

import frc.robot.bobot.Subsystem;

public class State extends Subsystem {
    public void apply() {

    }

    private State search(String name, State[] map) {
        for (State s : map) {
            if (s.getName().toLowerCase().equals(name.toLowerCase())) return s;
        }
        return null;
    }

    public State nextState(StateMachine.Input input, State[] stateMap) {
        return null;
    }
}
