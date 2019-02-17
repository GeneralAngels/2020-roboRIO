package frc.robot.efrat.statemachine.states;

import frc.robot.efrat.statemachine.State;
import frc.robot.efrat.statemachine.StateMachine;
import frc.robot.efrat.systems.Shanti;
import frc.robot.efrat.systems.Shiri;
import frc.robot.efrat.systems.rgb.RobotIdle;

public class InitState extends State {
    @Override
    public State nextState(StateMachine.Input input) {
        switch (input) {
            case OP_A:
                return find("cargoready");
            case OP_B:
                return find("hatchready");
            case DR_X:
                return find("climbz");
        }
        return null;
    }

    @Override
    public void apply() {
        progress("Init",0);
        RobotIdle.getInstance().idle();
        Shanti.getInstance().set(0.3, -0.3);
        while (!Shanti.getInstance().in_place(Shiri.getInstance().getCurrentX(), Shiri.getInstance().y, 0.5)) {
        }
        progress("Init",30);
        Shiri.getInstance().set(0);
        while (!Shiri.getInstance().in_place(Shanti.getInstance().getCurrentx(), 0.10)) {
        }
        progress("Init",70);
        Shanti.getInstance().set(0.65, -0.65);
        progress("Init",100);
    }
}
