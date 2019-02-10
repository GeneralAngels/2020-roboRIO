package frc.robot.efrat.statemachine;

import frc.robot.efrat.systems.rgb.RobotIdle;

import java.awt.*;

public class MyNadiState extends State {
    @Override
    public State nextState(StateMachine.Input input, State[] stateMap) {
        if (input == StateMachine.Input.OP_A)
            return search("initstate", stateMap);
        return null;
    }

    @Override
    public void apply() {
        RobotIdle.getInstance().color(Color.RED);
    }
}
