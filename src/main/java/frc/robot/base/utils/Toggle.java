package frc.robot.base.utils;

import frc.robot.base.Module;

public class Toggle extends Module {
    private boolean state = false;
    private boolean toggleState = false;
    private Change change;

    public Toggle(Change change) {
        this.change = change;
    }

    public void update(boolean newState) {
        if (newState != state) {
            state = newState;
            if (state) {
                toggleState = !toggleState;
                if (change != null) change.change(toggleState);
            }
        }
    }

    public void click() {
        log("Does this even work?");
        toggleState = !toggleState;
        if (change != null) change.change(toggleState);
    }

    public boolean getState() {
        return state;
    }

    public boolean getToggleState() {
        return toggleState;
    }

    public interface Change {
        void change(boolean toggle);
    }
}
