package frc.robot.bobot;

import edu.wpi.first.wpilibj.Timer;
import org.json.JSONObject;

public class Subsystem {
    private boolean logName = true;
    private String name = getClass().getSimpleName();

    public String getName() {
        return name;
    }

    private void setName(String name) {
        this.name = name;
    }

    public JSONObject toJSON() {
        return new JSONObject();
    }

    protected void doLogName() {
        logName = true;
    }

    protected void dontLogName() {
        logName = false;
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

    @Deprecated
    protected void delay(long millis) {
        try {
            Thread.sleep(millis);
        } catch (Exception e) {
            log("Failed to delay");
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
}
