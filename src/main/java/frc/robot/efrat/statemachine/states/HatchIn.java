package frc.robot.efrat.statemachine.states;

import frc.robot.efrat.statemachine.State;
import frc.robot.efrat.statemachine.StateMachine;
import frc.robot.efrat.systems.rgb.RobotIdle;

import java.awt.*;

public class HatchIn extends State {
    @Override
    public void apply() {
        RobotIdle.getInstance().color(new Color(255,50,0));
    }

    @Override
    public State nextState(StateMachine.Input input) {
        switch (input) {
            case OP_1:
                return find("hatchreadyL1shiri");
            case OP_2:
                return find("transferhatch1");
            case OP_3:
                return find("transferhatch1");
        }
        return null;
    }
}
