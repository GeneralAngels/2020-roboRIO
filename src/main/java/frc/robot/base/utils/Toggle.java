package frc.robot.base.utils;

import frc.robot.base.Module;
import org.json.JSONObject;

public class Toggle extends Module {

    private boolean graphState = false;
    private boolean toggleState = false;
    private OnStateChanged onStateChanged;

    public Toggle(OnStateChanged onStateChanged) {
        setOnStateChanged(onStateChanged);
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

    public void setOnStateChanged(OnStateChanged onStateChanged) {
        this.onStateChanged = onStateChanged;
    }

    public boolean getGraphState() {
        return graphState;
    }

    public boolean isToggled() {
        return toggleState;
    }

    @Override
    public JSONObject pullJSON() {
        JSONObject moduleJSON = super.pullJSON();
        moduleJSON.put("graph", getGraphState());
        moduleJSON.put("toggle", isToggled());
        return moduleJSON;
    }

    public interface OnStateChanged {
        void onStateChanged(boolean state);
    }
}
