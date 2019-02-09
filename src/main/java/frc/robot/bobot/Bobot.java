package frc.robot.bobot;

import org.json.JSONObject;

import java.util.ArrayList;

public class Bobot extends Subsystem {

    public static final String ACK = "ack";
    public static final String REQUEST = "request";
    public static final String DRIVE = "pneumaticDrive";
    public static final String SUBSYSTEMS = "subsystems";
    public static final String ROBOT_STATUS = "robot_status";
    public static final String DRIVER_STATUS = "driver_status";
    public static final String OPERATOR_STATUS = "operator_status";

    public static final String DEFAULT_VALUE = "{\"default\":\"If You Get This, Get Your Shit Together.\"}";

    protected ArrayList<Subsystem> subsystems = new ArrayList<>();

    protected TCP tcp;

    protected int id = 0;

    protected void addToJSON(Subsystem subsystem) {
        subsystems.add(subsystem);
    }

    protected void tcpInit() {
        tcp = new TCP();
        tcp.listen(newInput -> {

            if (newInput != null) {
                JSONObject input;
                try {
                    input = new JSONObject(newInput);
                } catch (Exception e) {
                    input = new JSONObject();
                }
                return handleJSON(input).toString();
            }
            return DEFAULT_VALUE;
        });
    }

    public void init() {
        tcpInit();
    }

    public void teleop() {

    }

    public void autonomous() {

    }

    @Override
    public JSONObject toJSON() {
        JSONObject returnObject = new JSONObject();
        try {
            JSONObject subsystemsObject = new JSONObject();
            for (Subsystem subsystem : subsystems) {
                try {
                    subsystemsObject.put(subsystem.getName().toLowerCase(), subsystem.toJSON());
                } catch (Exception ignored) {
                }
            }
            returnObject.put(SUBSYSTEMS, subsystemsObject);
        } catch (Exception ignored) {
        }
        return returnObject;
    }

    protected JSONObject handleJSON(JSONObject object) {
        return toJSON();
    }
}
