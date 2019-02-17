package frc.robot.efrat.statemachine.states;

import frc.robot.efrat.statemachine.State;
import frc.robot.efrat.statemachine.StateMachine;
import frc.robot.efrat.systems.Shanti;
import frc.robot.efrat.systems.Shiri;
import frc.robot.efrat.systems.rgb.RobotIdle;

import java.awt.*;

public class HatchReady extends State {
    @Override
    public void apply() {
        RobotIdle.getInstance().color(Color.MAGENTA);
        Shanti.getInstance().set(Shanti.getInstance().getCurrentx()-0.05,-0.4);
            while(!Shanti.getInstance().in_place(Shiri.getInstance().getCurrentX(),Shiri.getInstance().y,0.29)){}
            Shiri.getInstance().set(0.5);
            while(!Shiri.getInstance().in_place(Shanti.getInstance().getCurrentx(),0.09)){}
            log("gothere");
            Shanti.getInstance().set(0.6,-0.5);
        }

        @Override
        public State nextState(StateMachine.Input input) {
//        if (Shiri.getInstance().isHatchLoaded()) {
            if (false){
            return find("hatchin");
        } else switch (input) {
            case OP_A:
                return find("cargoready");
        }
        return null;
    }
}
