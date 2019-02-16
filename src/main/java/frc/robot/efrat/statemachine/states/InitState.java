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
//        RobotIdle.getInstance().idle();
        Shiri.getInstance().set(0);
        while (!Shiri.getInstance().in_place()){}
//        log("tom");
        Shanti.getInstance().set(0.75,-0.9);

    }
}
