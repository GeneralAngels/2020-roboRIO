package frc.robot.bobot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.json.JSONObject;

import java.util.ArrayList;

public class Bobot extends Subsystem {
    public static final String TIME = "time";
    public static final String SUBSYSTEMS = "subsystems";
    public static final String ROBOT_STATUS = "robot_status";
    public static final String DRIVER_STATUS = "driver_status";
    public static final String OPERATOR_STATUS = "operator_status";

    public static final String DEFAULT_VALUE = "{\"default\":\"If You Get This, Get Your Things Together.\"}";

    public static final int LOG_FREQUENCY = 25;

    protected NetworkTableInstance nti;
    protected NetworkTable database;
    protected NetworkTableEntry json;

    protected ArrayList<Subsystem> subsystems = new ArrayList<>();

    protected TCP tcp;

    protected int loop = 0;

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
        nti = NetworkTableInstance.getDefault();
        database = nti.getTable("database");
        json = database.getEntry("json");
        tcpInit();
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
        JSONObject returnObject = new JSONObject();
        returnObject.put(TIME, millis());
        try {
            for (Subsystem subsystem : subsystems) {
                try {
                    returnObject.put(subsystem.getName().toLowerCase(), subsystem.toJSON());
                } catch (Exception ignored) {
                }
            }
        } catch (Exception ignored) {
        }
        return returnObject;
    }

    protected JSONObject handleJSON(JSONObject object) {
        return toJSON();
    }
}
