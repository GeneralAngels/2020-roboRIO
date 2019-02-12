package frc.robot.efrat.statemachine.states;

import frc.robot.efrat.statemachine.State;
import frc.robot.efrat.statemachine.StateMachine;
import frc.robot.efrat.systems.Shanti;
import frc.robot.efrat.systems.rgb.RobotIdle;

import java.awt.*;

public class HatchReadyL3 extends State {
    @Override
    public void apply() {
//        Shanti.getInstance().set(Shanti.LVL3_X,Shanti.LVL3_Y);
    }

    @Override
    public State nextState(StateMachine.Input input) {
        switch (input) {
            case OP_1:
                return find("hatchreadyL1");
            case OP_2:
                return find("hatchreadyL2");
            case OP_3:
                return find("hatchoutL3");
        }
        return null;
    }
}
