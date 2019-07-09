package frc.robot.base;

import frc.robot.base.communications.Communicator;
import org.json.JSONObject;

public class Bot extends Module {
    private static final String TIME = "time";

    private Communicator communicator = null;

    public void init() {
        communicator = new Communicator(this);
    }

    public void teleop() {
        if (communicator != null) communicator.update();
    }

    public void autonomous() {
        if (communicator != null) communicator.update();
    }

    @Override
    public JSONObject pullJSON() {
        JSONObject moduleJSON = super.pullJSON();
        moduleJSON.put(TIME, millis());
        return moduleJSON;
    }
}
