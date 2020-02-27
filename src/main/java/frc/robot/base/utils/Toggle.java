package frc.robot.base.utils;

import com.ga2230.shleam.advanced.frc.FRCModule;

/**
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

public class Toggle extends FRCModule {

    private boolean pushedState = false;
    private boolean toggleState = false;
    private OnStateChanged onStateChanged;

    public Toggle(String id, OnStateChanged onStateChanged) {
        super(id);
        setOnStateChanged(onStateChanged);
    }

    public void update(boolean buttonInput) {
        if (pushedState != buttonInput) {
            pushedState = buttonInput;
            if (pushedState) {
                toggleState = !toggleState;
                if (onStateChanged != null) {
                    onStateChanged.onStateChanged(toggleState);
                }
            }
        }
        set("toggle", String.valueOf(isToggled()));
        set("pushed", String.valueOf(getPushedState()));
    }

    public void setOnStateChanged(OnStateChanged onStateChanged) {
        this.onStateChanged = onStateChanged;
    }

    public boolean getPushedState() {
        return pushedState;
    }

    public boolean isToggled() {
        return toggleState;
    }

    public interface OnStateChanged {
        void onStateChanged(boolean state);
    }
}
