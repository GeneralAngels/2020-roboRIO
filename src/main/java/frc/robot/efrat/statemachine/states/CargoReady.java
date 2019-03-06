package frc.robot.efrat.statemachine.states;

import frc.robot.efrat.statemachine.State;
import frc.robot.efrat.statemachine.StateMachine;
import frc.robot.efrat.systems.Shanti;
import frc.robot.efrat.systems.Shiri;
import frc.robot.efrat.systems.Tomer;
import frc.robot.efrat.systems.rgb.RobotIdle;

import java.awt.*;

public class CargoReady extends State {
    @Override
    public void apply() {
        progress("Prep For Cargo",0);
        RobotIdle.getInstance().color(Color.MAGENTA);
        Shanti.getInstance().set(Shanti.getInstance().getCurrentx(), -0.3);
        while (!Shanti.getInstance().in_place(Shiri.getInstance().getCurrentX(), Shiri.getInstance().y, 0.29)) {
        }
        progress("Prep For Cargo",50);
        Shiri.getInstance().set(0);
//        while(!Shiri.getInstance().in_place())
//        Roller.getInstance().open();
        Shanti.getInstance().set(0.7, -0.65);
        progress("Prep For Cargo",100);
    }

    @Override
    public State nextState(StateMachine.Input input) {
        if (Tomer.getInstance().isCargoLoaded()) {
            return find("cargoin");
        } else switch (input) {
            case OP_B:
                return find("hatchready");
        }
        return null;
    }
}
