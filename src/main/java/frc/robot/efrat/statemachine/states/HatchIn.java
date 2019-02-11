package frc.robot.efrat.statemachine.states;

import frc.robot.efrat.statemachine.State;
import frc.robot.efrat.statemachine.StateMachine;
import frc.robot.efrat.systems.rgb.RobotIdle;

import java.awt.*;

public class HatchIn extends State {
    @Override
    public void apply() {
        RobotIdle.getInstance().flash(Color.YELLOW);
    }

    @Override
    public State nextState(StateMachine.Input input) {
        return null;
    }
}
