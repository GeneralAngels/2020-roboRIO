package frc.robot.base;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.base.communications.Communicator;
import org.json.JSONObject;

import java.util.ArrayList;

public class Bot extends Module {
    public static final String TIME = "time";

    protected NetworkTableInstance nti;
    protected NetworkTable database;
    protected NetworkTableEntry json;
    protected Communicator communicator;

    protected ArrayList<Module> modules = new ArrayList<>();

    protected void addToJSON(Module module) {
        modules.add(module);
    }

    public void init() {
        communicator = new Communicator(this);
    }

    public void teleop() {
        loop();
    }

    public void autonomous() {
        loop();
    }

    protected void loop() {
        json.setString(toJSON().toString());
    }

    @Override
    public JSONObject toJSON() {
        JSONObject returnObject = super.toJSON();
        returnObject.put(TIME, millis());
        try {
            for (Module module : modules) {
                try {
                    returnObject.put(module.getName().toLowerCase(), module.toJSON());
                } catch (Exception ignored) {
                }
            }
        } catch (Exception ignored) {
        }
        return returnObject;
    }

    protected void handleJSON(JSONObject object) {
    }
}
