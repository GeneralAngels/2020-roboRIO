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

    private static final String WHITESPACE = "\t";

    public Module(String id) {
        super(id);
        command("json", s -> pullJSON(false).toString());
        command("telemetry", s -> pullJSON(true).toString());
        command("help", s -> help(""));
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

    public String help(String prependingWhitespace) {
        StringBuilder builder = new StringBuilder();
        builder.append(prependingWhitespace).append(":").append(" ").append(id.toLowerCase()).append("\r\n");
        getCommands().forEach((s, command) -> {
            builder.append(prependingWhitespace).append(WHITESPACE).append(">").append(" ").append(s).append("\r\n");
        });
        for (Node slav : super.getSlaves()) {
            builder.append(((Module) slav).help(prependingWhitespace + WHITESPACE));
        }
        return builder.toString();
    }

    protected void log(String string) {
        System.out.println(id.toUpperCase() + ": " + string);
    }

    protected long millis() {
        return (long) (Timer.getFPGATimestamp() * 1000);
    }

}
