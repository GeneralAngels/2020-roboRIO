package frc.robot.efrat.statemachine;

import frc.robot.bobot.Subsystem;
import frc.robot.efrat.systems.rgb.RobotIdle;

import java.awt.*;

public class State extends Subsystem {
    public void apply() {

    }

    protected State find(String name) {
        for (State s : StateMachine.getStateMap()) {
            if (s.getName().toLowerCase().equals(name.toLowerCase())) return s;
        }
        return null;
    }

    public State nextState(StateMachine.Input input) {
        return null;
    }
}
