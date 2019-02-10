package frc.robot.efrat.statemachine;

import edu.wpi.first.wpilibj.XboxController;

public class StateMachine {
    public State[] stateMap = {};
    public State lastState;
    public State currentState;

    public void update(XboxController op, XboxController dr) {

    }

    public enum Input {
        NONE,
        OP_A,
        OP_B,
        OP_X,
        OP_Y,
        OP_DPAD_UP,
        OP_DPAD_DOWN,
        OP_DPAD_LEFT,
        OP_DPAD_RIGHT
    }
}
