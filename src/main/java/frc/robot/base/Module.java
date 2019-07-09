package frc.robot.base;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.base.utils.Tuple;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.Map;

public class Module {
    // Internal Log-Objects
    private ArrayList<Tuple<String, Object>> objects = new ArrayList<>();
    // Sub-modules
    private ArrayList<Module> modules = new ArrayList<>();

    protected void register(Module module) {
        modules.add(module);
    }

    protected void unregister(Module module) {
        modules.remove(module);
    }

    public String getName() {
        return getClass().getSimpleName();
    }

    public JSONObject pullJSON() {
        JSONObject json = new JSONObject();
        JSONObject objectsJSON = new JSONObject();
        JSONObject modulesJSON = new JSONObject();
        for (Tuple<String, Object> object : objects) {
            if (object != null) {
                if (object.getSecond() != null)
                    objectsJSON.put(object.getFirst(), object.getSecond().toString());
            }
        }
        for (Module module : modules) {
            if (module != null) {
                modulesJSON.put(module.getName(), module.pullJSON());
            }
        }
        json.put("objects", objectsJSON);
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

    protected void log(String name, Object value) {
        for (Tuple<String, Object> object : objects) {
            if (object.getFirst().equals(name)) {
                object.setSecond(value);
                return;
            }
        }
        objects.add(new Tuple<>(name, value));
    }

    protected void log(String string) {
        System.out.println(getName() + ": " + string);
    }

    protected long millis() {
        return (long) (Timer.getFPGATimestamp() * 1000);
    }
}
