package frc.robot.efrat.statemachine.states;

import frc.robot.efrat.statemachine.State;
import frc.robot.efrat.statemachine.StateMachine;
import frc.robot.efrat.systems.rgb.RobotIdle;

import java.awt.*;

public class CargoReadyL3 extends State {
    @Override
    public void apply() {
    }

    @Override
    public State nextState(StateMachine.Input input) {
        switch (input){
            case OP_Y:
                return find("cargooutL3");
            case OP_1:
                return find("cargoreadyL1");
            case OP_2:
                return find("cargoreadyL2");
        }
        return null;
    }
}
