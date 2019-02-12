package frc.robot.efrat.statemachine.states;

import frc.robot.efrat.statemachine.State;
import frc.robot.efrat.statemachine.StateMachine;
import frc.robot.efrat.systems.Shiri;
import frc.robot.efrat.systems.Tomer;

public class TransferHatch2 extends State {
    @Override
    public void apply() {
        progress("Disconnect Shiri", 0);
        Shiri.getInstance().moveToBack();
        while (!Shiri.getInstance().isAtBack()) ;
        progress("Disconnect Shiri", 50);
        Tomer.getInstance().lift();
        while (Tomer.getInstance().isDown()) ;
        progress("Disconnect Shiri", 100);
    }

    @Override
    public State nextState(StateMachine.Input input) {
        switch (input) {
            case OP_1:
                return find("hatchreadyL1");
            case OP_2:
                return find("hatchreadyL2");
            case OP_3:
                return find("hatchreadyL3");
        }
        return null;
    }
}
