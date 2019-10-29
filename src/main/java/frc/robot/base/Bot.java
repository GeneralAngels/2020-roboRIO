/*
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

package frc.robot.base;

import com.ga2230.networking.Server;
import org.json.JSONObject;

public class Bot extends Module {

    private String lastPull = "";
    private String currentPull = "";

    public void init() {
        Server.begin((s, dialog) -> {
            if (s.length() > 0)
                pushJSON(new JSONObject(s));
        });
    }

    public void teleop() {
        communicate();
    }

    public void autonomous() {
        communicate();
    }

    private void communicate() {
        currentPull = pullJSON().toString();
        if (!currentPull.equals(lastPull)) {
            Server.send(currentPull);
            lastPull = currentPull;
        }
    }
}
