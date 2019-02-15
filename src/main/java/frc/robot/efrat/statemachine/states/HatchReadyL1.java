package frc.robot.efrat.statemachine.states;

import frc.robot.efrat.statemachine.State;
import frc.robot.efrat.statemachine.StateMachine;
import frc.robot.efrat.systems.rgb.RobotIdle;

import java.awt.*;

public class HatchReadyL1 extends State {
    @Override
    public void apply() {
        // TODO move shiri to back
    }

    @Override
    public State nextState(StateMachine.Input input) {
        switch (input) {
            case OP_Y:
                return find("hatchoutL1");
            case OP_2:
                return find("hatchreadyL2");
            case OP_3:
                return find("hatchreadyL3");
        }
        return null;
    }
}
