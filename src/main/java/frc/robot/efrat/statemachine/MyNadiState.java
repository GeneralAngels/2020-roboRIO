package frc.robot.efrat.statemachine;

import frc.robot.efrat.systems.Shiri;

public class MyNadiState extends State {
    @Override
    public State nextState(StateMachine.Input input, State[] stateMap) {
        return search("initstate",stateMap);
    }

    @Override
    public void apply() {
        Shiri.getInstance().open();
    }
}
