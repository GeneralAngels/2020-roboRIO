package frc.robot.base.utils;

import frc.robot.base.Module;

public class Toggle extends Module {

    private boolean graphState = false;
    private boolean toggleState = false;
    private OnStateChanged onStateChanged;

    public Toggle(OnStateChanged onStateChanged) {
        this.onStateChanged = onStateChanged;
    }

    public void update(boolean buttonInput) {
        if (graphState != buttonInput) {
            graphState = buttonInput;
            if (graphState) {
                toggleState = !toggleState;
                if (onStateChanged != null) {
                    onStateChanged.onStateChanged(toggleState);
                }
            }
        }
    }

    public boolean getGraphState() {
        return graphState;
    }

    public boolean getToggleState() {
        return toggleState;
    }

    public interface OnStateChanged {
        void onStateChanged(boolean state);
    }
}
