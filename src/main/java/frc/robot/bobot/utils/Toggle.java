package frc.robot.bobot.utils;

import frc.robot.bobot.Subsystem;

public class Toggle extends Subsystem {
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

    public boolean getToggleState() {
        return toggleState;
    }

    public interface Change {
        void change(boolean toggle);
    }
}
