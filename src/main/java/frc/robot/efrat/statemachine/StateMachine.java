package frc.robot.efrat.statemachine;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.bobot.Subsystem;
import frc.robot.bobot.utils.Toggle;
import frc.robot.efrat.statemachine.states.*;
import frc.robot.efrat.systems.Roller;
import frc.robot.efrat.systems.Shanti;
import frc.robot.efrat.systems.Shiri;
import frc.robot.efrat.systems.Tomer;
import frc.robot.efrat.systems.rgb.RobotIdle;
import org.json.JSONObject;

import java.awt.*;

public class StateMachine extends Subsystem {
    public static final String CURRENT_STATE = "current_state";
    // TODO to add states, add NEW YOURSTATE() to the array
    private static final State[] stateMap = {
            new InitState(),
            new CargoReady(),
            new HatchReady(),
            new CargoIn(),
            new HatchIn(),
            new CargoReadyL1(),
            new CargoReadyL2(),
            new CargoReadyL3(),
            new CargoOutL1(),
            new CargoOutL2(),
            new CargoOutL3(),
            new HatchReadyL1(),
            new HatchReadyL2(),
            new HatchReadyL3(),
            new HatchOutL1(),
            new HatchOutL2(),
            new HatchOutL3(),
            new TransferHatch1(),
            new TransferHatch2(),
            new HatchReadyL1Shiri(),
            new HatchOutL1Shiri(),
            new ClimbZ()
    };
    private State lastState, currentState;
    private Input currentInput = Input.NONE;
    private Toggle drA, drB, drX, drY, dr1, dr2, dr3, dr4;
    private Toggle opA, opB, opX, opY, op1, op2, op3, op4;

    public StateMachine() {
        currentState = stateMap[0];
        initToggles();
        initSubsystems();
    }

    public static State[] getStateMap() {
        return stateMap;
    }

    private void initSubsystems() {
        Tomer.init();
        Shiri.init();
        Shanti.init();
        Roller.init();
    }

    private void initToggles() {
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
        new Thread(() -> {
            currentInput = Input.NONE;
            updateToggles(op, dr);
            if (currentInput != Input.NONE) {
                if (currentState != null) {
                    lastState = currentState;
                    currentState = currentState.nextState(currentInput);
                    if (currentState == null) {
                        currentState = lastState;
                        RobotIdle.getInstance().color(Color.RED);
                    } else {
                        currentState.apply();
                    }
                    notifyChange();
                } else {
                    RobotIdle.getInstance().flash(Color.RED);
                    log("No State");
                }
            } else {
                if (currentState != null && currentState.nextState(Input.NONE) != null && currentState != currentState.nextState(Input.NONE)) {
                    lastState = currentState;
                    currentState = currentState.nextState(currentInput);
                    currentState.apply();
                    notifyChange();
                }
            }
        }).start();
    }

    private void notifyChange() {
        log((lastState != null ? lastState.getName() : "none") + " to " + (currentState != null ? currentState.getName() : "none"));
    }

    private void updateToggles(XboxController op, XboxController dr) {
        update(op, opA, opB, opX, opY, op1, op2, op3, op4);
        update(dr, drA, drB, drX, drY, dr1, dr2, dr3, dr4);
    }

    private void update(XboxController controller, Toggle A, Toggle B, Toggle X, Toggle Y, Toggle D1, Toggle D2, Toggle D3, Toggle D4) {
        if (controller != null) {
            A.update(controller.getAButton());
            B.update(controller.getBButton());
            X.update(controller.getXButton());
            Y.update(controller.getYButton());
            D1.update(controller.getPOV() == 180);
            D2.update(controller.getPOV() == 90);
            D3.update(controller.getPOV() == 0);
            D4.update(controller.getPOV() == 270);
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
