package frc.robot.efrat.statemachine.states;

import frc.robot.efrat.statemachine.State;
import frc.robot.efrat.statemachine.StateMachine;

public class MyNadiState extends State {
    @Override
    public State nextState(StateMachine.Input input) {
        if (input == StateMachine.Input.OP_A)
            return find("initstate");
        return null;
    }

    @Override
    public void apply() {
//        Stick.getInstance().set();
    }
}
