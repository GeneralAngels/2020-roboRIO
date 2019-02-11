package frc.robot.efrat.statemachine.states;

import frc.robot.efrat.statemachine.State;
import frc.robot.efrat.statemachine.StateMachine;

public class HatchOutL3 extends State {
    @Override
    public void apply() {
    }

    @Override
    public State nextState(StateMachine.Input input) {
        switch (input) {
            case OP_A:
                return find("cargoready");
            case OP_B:
                return find("hatchready");
        }
        return null;
    }
}
