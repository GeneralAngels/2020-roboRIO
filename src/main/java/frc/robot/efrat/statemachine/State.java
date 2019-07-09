package frc.robot.efrat.statemachine;

import frc.robot.base.Module;

public class State extends Module {
    public void apply() {

    }

    protected State find(String name) {
        for (State s : StateMachine.getStateMap()) {
            if (s.getName().toLowerCase().equals(name.toLowerCase())) return s;
        }
        return null;
    }

    public State nextState(StateMachine.Input input) {
        return null;
    }
}
