package frc.robot.efrat.statemachine.states;

import frc.robot.efrat.statemachine.State;
import frc.robot.efrat.statemachine.StateMachine;
import frc.robot.efrat.systems.Stick;
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
//        Stick.getInstance().set();
    }
}
