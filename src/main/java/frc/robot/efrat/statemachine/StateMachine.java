package frc.robot.efrat.statemachine;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.bobot.Subsystem;
import frc.robot.bobot.utils.Toggle;
import org.json.JSONObject;

public class StateMachine extends Subsystem {
    public static final String CURRENT_STATE = "current_state";
    private State[] stateMap = {new InitState(), new MyNadiState()};
    private State tempState;
    private State currentState;
    private Input currentInput = Input.NONE;
    private Toggle drA, drB, drX, drY, dr1, dr2, dr3, dr4;
    private Toggle opA, opB, opX, opY, op1, op2, op3, op4;

    public StateMachine() {
        currentState = stateMap[0];
        drA = new Toggle(toggle -> currentInput = Input.DR_A);
        drB = new Toggle(toggle -> currentInput = Input.DR_B);
        drX = new Toggle(toggle -> currentInput = Input.DR_X);
        drY = new Toggle(toggle -> currentInput = Input.DR_Y);
        dr1 = new Toggle(toggle -> currentInput = Input.DR_1);
        dr2 = new Toggle(toggle -> currentInput = Input.DR_2);
        dr3 = new Toggle(toggle -> currentInput = Input.DR_3);
        dr4 = new Toggle(toggle -> currentInput = Input.DR_4);
        opA = new Toggle(toggle -> currentInput = Input.OP_A);
        opB = new Toggle(toggle -> currentInput = Input.OP_B);
        opX = new Toggle(toggle -> currentInput = Input.OP_X);
        opY = new Toggle(toggle -> currentInput = Input.OP_Y);
        op1 = new Toggle(toggle -> currentInput = Input.OP_1);
        op2 = new Toggle(toggle -> currentInput = Input.OP_2);
        op3 = new Toggle(toggle -> currentInput = Input.OP_3);
        op4 = new Toggle(toggle -> currentInput = Input.OP_4);
    }

    public void update(XboxController op, XboxController dr) {
        currentInput = Input.NONE;
        if (op != null) {
            opA.update(op.getAButton());
            opB.update(op.getBButton());
            opX.update(op.getXButton());
            opY.update(op.getYButton());
            op1.update(op.getPOV() == 0);
            op2.update(op.getPOV() == 90);
            op3.update(op.getPOV() == 180);
            op4.update(op.getPOV() == 270);
        }
        if (dr != null) {
            drA.update(dr.getAButton());
            drB.update(dr.getBButton());
            drX.update(dr.getXButton());
            drY.update(dr.getYButton());
            dr1.update(dr.getPOV() == 0);
            dr2.update(dr.getPOV() == 90);
            dr3.update(dr.getPOV() == 180);
            dr4.update(dr.getPOV() == 270);
        }
        if (currentState != null) {
            tempState = currentState.nextState(currentInput, stateMap);
            if (tempState != null) {
                currentState = tempState;
                currentState.apply();
            }
        }
    }

    @Override
    public JSONObject toJSON() {
        JSONObject json = new JSONObject();
        if (currentState != null)
            json.put(CURRENT_STATE, currentState.getName());
        else
            json.put(CURRENT_STATE, "none");
        return json;
    }

    public enum Input {
        NONE,
        OP_A,
        OP_B,
        OP_X,
        OP_Y,
        OP_1,
        OP_2,
        OP_3,
        OP_4,
        DR_A,
        DR_B,
        DR_X,
        DR_Y,
        DR_1,
        DR_2,
        DR_3,
        DR_4,
    }
}
