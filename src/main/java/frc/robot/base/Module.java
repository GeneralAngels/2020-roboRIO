/*
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

package frc.robot.base;

import com.ga2230.networking.Node;
import edu.wpi.first.wpilibj.Timer;
import org.json.JSONObject;

public class Module extends Node {

    public Module(String id) {
        super(id);
        addCommand("getjson", s -> pullJSON().toString());
    }

    public JSONObject pullJSON() {
        JSONObject json = new JSONObject();
        variables.forEach(json::put);
        return json;
    }

    protected void log(String string) {
        System.out.println(id.toUpperCase() + ": " + string);
    }

    protected long millis() {
        return (long) (Timer.getFPGATimestamp() * 1000);
    }

}
