package frc.robot.efrat.statemachine.states;

import frc.robot.efrat.statemachine.State;
import frc.robot.efrat.statemachine.StateMachine;
import frc.robot.efrat.systems.Fork;
import frc.robot.efrat.systems.rgb.RobotIdle;

import java.awt.*;

public class CargoReady extends State {
    @Override
    public void apply() {
        RobotIdle.getInstance().color(Color.MAGENTA);
    }

    @Override
    public State nextState(StateMachine.Input input) {
        if (Fork.getInstance().isCargoLoaded()) {
            return find("cargoin");
        } else switch (input) {
            case OP_B:
                return find("hatchready");
        }
        return null;
    }
}
