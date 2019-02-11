package frc.robot.efrat.statemachine.states;

import frc.robot.efrat.statemachine.State;
import frc.robot.efrat.statemachine.StateMachine;
import frc.robot.efrat.systems.rgb.RobotIdle;

import java.awt.*;

public class InitState extends State {
    @Override
    public State nextState(StateMachine.Input input) {
        switch (input){
            case OP_A:
                return find("cargoready");
        }
        return null;
    }

    @Override
    public void apply() {
        RobotIdle.getInstance().color(Color.YELLOW);
    }
}
