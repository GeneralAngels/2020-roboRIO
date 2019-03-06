package frc.robot.efrat.statemachine.states;

import frc.robot.efrat.statemachine.State;
import frc.robot.efrat.statemachine.StateMachine;

public class HatchReadyL1Shiri extends State {
    @Override
    public void apply() {
        // TODO shiri go to front
    }

    @Override
    public State nextState(StateMachine.Input input) {
        switch (input) {
            case OP_Y:
                return find("hatchoutL1shiri");
            case OP_2:
                return find("transferhatch1");
            case OP_3:
                return find("transferhatch1");
        }
        return null;
    }
}
