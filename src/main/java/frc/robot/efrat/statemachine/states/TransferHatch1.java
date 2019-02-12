package frc.robot.efrat.statemachine.states;

import frc.robot.efrat.statemachine.State;
import frc.robot.efrat.statemachine.StateMachine;
import frc.robot.efrat.systems.Shanti;
import frc.robot.efrat.systems.Shiri;
import frc.robot.efrat.systems.Tomer;

public class TransferHatch1 extends State {
    @Override
    public void apply() {
        progress("Transfer Hatch", 0);
        Shiri.getInstance().moveToFront();
        while (!Shiri.getInstance().isAtFront()) ;
        progress("Transfer Hatch", 20);
        Tomer.getInstance().drop();
        while (!Tomer.getInstance().isDown()) ;
        progress("Transfer Hatch", 40);
        Tomer.getInstance().close();
        while (Tomer.getInstance().isOpened()) ;
        progress("Transfer Hatch", 60);
        Shanti.getInstance().moveToExchange();
        while (!Shanti.getInstance().isAtExchange()) ;
        progress("Transfer Hatch", 80);
        Tomer.getInstance().open();
        while (!Tomer.getInstance().isOpened());
        progress("Transfer Hatch", 100);

    }

    private boolean primeState() {
        return Shiri.getInstance().isAtFront() && !Shiri.getInstance().isHatchLocked() && Shanti.getInstance().isAtExchange() && Tomer.getInstance().isDown() && !Tomer.getInstance().isOpened();
    }

    @Override
    public State nextState(StateMachine.Input input) {
        if (primeState()) {
            find("transferhatch2");
        }
        return null;
    }
}
