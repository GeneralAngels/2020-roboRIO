package frc.robot.efrat.statemachine.states;

import frc.robot.efrat.statemachine.State;
import frc.robot.efrat.statemachine.StateMachine;
import frc.robot.efrat.systems.rgb.RobotIdle;

import java.awt.*;

public class HatchReadyL1Shiri extends State {
    @Override
    public void apply() {
        // TODO shiri go to front
    }

    @Override
    public State nextState(StateMachine.Input input) {
        switch (input) {
            case OP_X:
                return find("hatchoutL1shiri");
            case OP_2:
                return find("transferhatch1");
            case OP_3:
                return find("transferhatch1");
        }
        return null;
    }
}
