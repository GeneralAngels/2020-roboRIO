/*
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

package frc.robot.base;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.base.communication.Node;
import org.json.JSONObject;

public class Module extends Node {

    private static final String WHITESPACE = "\t";

    protected boolean execute = true;

    public Module(String id) {
        super(id);
        command("json", s -> new Tuple<>(true, pullJSON(false).toString()));
        command("telemetry", s -> new Tuple<>(true, pullJSON(true).toString()));
        command("help", s -> new Tuple<>(true, help("")));
        command("sleep", new Command() {

            private boolean sleeping = false;
            private long targetTime = 0;

            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                if (!sleeping) {
                    int time = Integer.parseInt(s);
                    // Set target time
                    targetTime = millis() + time;
                    // Sleeping switch
                    sleeping = true;
                } else {
                    if (millis() > targetTime) {
                        sleeping = false;
                        // Return done
                        return new Tuple<>(true, "Waited");
                    }
                }
                return new Tuple<>(false, "Sleeping");
            }
        });

        command("log", s -> {
            log(s);
            return new Tuple<>(true, "Logged");
        });

        command("watch", new Command() {

            private long offset = 0;

            @Override
            public Tuple<Boolean, String> execute(String parameter) throws Exception {
                if (parameter.equals("start")) {
                    offset = millis();
                } else {
                    long current = millis();
                    log("WATCH: " + (current - offset) + "ms, " + ((current - offset) / 1000) + "s.");
                }
                return new Tuple<>(true, "Handled");
            }
        });
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

    public void setExecute(boolean execute) {
        this.execute = execute;
    }

    public boolean getExecute() {
        return execute;
    }

    @Override
    public Tuple<Boolean, String> execute(String command, String parameter) throws Exception {
        if (execute)
            return super.execute(command, parameter);
        return null;
    }
}
