package frc.robot.bobot;

import edu.wpi.first.wpilibj.Timer;
import org.json.JSONObject;

import java.util.ArrayList;

public class Subsystem {
    private boolean logName = false;
    private String name = getClass().getSimpleName();
    private ArrayList<Logable> logables = new ArrayList<>();

    public String getName() {
        return name;
    }

    private void setName(String name) {
        this.name = name;
    }

    public JSONObject toJSON() {
        JSONObject json = new JSONObject();
        for (Logable logable : logables) json.put(logable.name, logable.value.toString());
        return json;
    }

    protected void doLogName() {
        logName = true;
    }

    protected void dontLogName() {
        logName = false;
    }

    protected void log(String name, Object value) {
        for (Logable logable : logables) {
            if (logable.name.equals(name)) {
                logable.value = value;
                return;
            }
        }
        logables.add(new Logable(name, value));
    }

    protected void log(String string) {
        System.out.println((logName ? getName() + ": " : "") + string);
    }

    private String create(char c, int length) {
        if (length > 0) {
            return c + create(c, length - 1);
        }
        return "";
    }

    protected void progress(String process, double progress) {
        if (progress <= 100) {
            dontLogName();
            int barLength = (int) (progress / 10.0);
            String toPrint = "[" + getName() + "->" + process + "," + (progress < 100 ? " " : "") + ((progress < 10) ? "0" : "") + (int) progress + "%] [" + create('#', barLength) + create('_', 10 - barLength) + "]";
            log(toPrint);
            doLogName();
        }
    }

    protected long millis() {
        return (long) (Timer.getFPGATimestamp() * 1000);
    }

    protected long micros() {
        return (long) (Timer.getFPGATimestamp() * 1000000);
    }

    protected boolean passed(long milli) {
        return millis() > milli;
    }

    private class Logable {
        private Object value;
        private String name;

        private Logable(String name, Object value) {
            this.value = value;
            this.name = name;
        }
    }
}
