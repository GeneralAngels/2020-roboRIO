package frc.robot.efrat.statemachine.states;

import frc.robot.efrat.statemachine.State;
import frc.robot.efrat.statemachine.StateMachine;
import frc.robot.efrat.systems.Shiri;
import frc.robot.efrat.systems.rgb.RobotIdle;

import java.awt.*;

public class HatchReady extends State {
    @Override
    public void apply() {
        RobotIdle.getInstance().color(Color.MAGENTA);
    }

    @Override
    public State nextState(StateMachine.Input input) {
        if (Shiri.getInstance().isHatchLoaded()) {
            return find("hatchin");
        } else switch (input) {
            case OP_A:
                return find("cargoready");
        }
        return null;
    }
}
