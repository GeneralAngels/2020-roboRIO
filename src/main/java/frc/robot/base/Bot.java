/*
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

package frc.robot.base;

import com.ga2230.networking.Server;
import org.json.JSONObject;

public class Bot extends Module {
    public void init() {
        Server.begin((s, dialog) -> pushJSON(new JSONObject(s)));
    }

    public void teleop() {
        Server.send(pullJSON().toString());
    }

    public void autonomous() {
        Server.send(pullJSON().toString());
    }
}
