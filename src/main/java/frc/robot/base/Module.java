/*
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

package frc.robot.base;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.base.utils.Tuple;
import org.json.JSONObject;

import java.util.ArrayList;

public class Module {
    // Internal Log-Objects
    private ArrayList<Tuple<String, Object>> values = new ArrayList<>();
    // Sub-modules
    private ArrayList<Module> modules = new ArrayList<>();
    // JSON ID
    private String id = null;

    protected void register(Module module) {
        modules.add(module);
    }

    protected void unregister(Module module) {
        modules.remove(module);
    }

    public String getName() {
        return getClass().getSimpleName().toLowerCase();
    }

    public JSONObject pullJSON() {
        JSONObject json = new JSONObject();
        JSONObject objectsJSON = new JSONObject();
        JSONObject modulesJSON = new JSONObject();
        for (Tuple<String, Object> object : values) {
            if (object != null) {
                if (object.getSecond() != null)
                    objectsJSON.put(object.getFirst(), object.getSecond());
            }
        }
        for (Module module : modules) {
            if (module != null) {
                modulesJSON.put(module.getID(), module.pullJSON());
            }
        }
        json.put("values", objectsJSON);
        json.put("modules", modulesJSON);
        return json;
    }

    public void pushJSON(JSONObject object) {
        try {
            for (Module module : modules) {
                if (module != null) {
                    if (object.has(module.getName())) {
                        module.pushJSON(object.getJSONObject(module.getName()));
                    }
                }
            }
        } catch (Exception e) {
            log("Exception thrown in pushJSON, " + e.getMessage());
        }
    }

    public void setID(String id) {
        this.id = id;
    }

    public String getID() {
        return this.id != null ? this.id : getName();
    }

    protected void log(String name, Object value) {
        for (Tuple<String, Object> object : values) {
            if (object.getFirst().equals(name)) {
                object.setSecond(value);
                return;
            }
        }
        values.add(new Tuple<>(name, value));
    }

    protected void log(String string) {
        System.out.println(getName() + ": " + string);
    }

    protected long millis() {
        return (long) (Timer.getFPGATimestamp() * 1000);
    }

}
