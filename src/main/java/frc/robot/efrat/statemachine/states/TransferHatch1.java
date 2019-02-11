package frc.robot.efrat.statemachine.states;

import frc.robot.efrat.statemachine.State;
import frc.robot.efrat.statemachine.StateMachine;

public class TransferHatch1 extends State {
    @Override
    public void apply() {
    }

    @Override
    public State nextState(StateMachine.Input input) {
        // TODO if shanti is down and shiri is extended find(transferhatch2)
        return find("transferhatch2");
    }
}
