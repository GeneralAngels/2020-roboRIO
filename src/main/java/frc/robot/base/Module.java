/*
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

package frc.robot.base;

import com.ga2230.networking.Node;
import edu.wpi.first.wpilibj.Timer;
import org.json.JSONArray;
import org.json.JSONObject;

public class Module extends Node {

    public Module(String id) {
        super(id);
        command("json", s -> pullJSON(false).toString());
        command("telemetry", s -> pullJSON(true).toString());
        command("exists", s -> "true");
    }

    public JSONObject pullJSON(boolean recursive) {
        JSONObject json = new JSONObject();
        super.variables.forEach(json::put);
        if (recursive) {
            for (Node slave : super.getSlaves()) {
                json.put(slave.getID(), ((Module) slave).pullJSON(recursive));
            }
        }
        return json;
    }

    protected void log(String string) {
        System.out.println(id.toUpperCase() + ": " + string);
    }

    protected long millis() {
        return (long) (Timer.getFPGATimestamp() * 1000);
    }

}
