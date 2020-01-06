/*
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

package frc.robot.base;

import com.ga2230.networking.Server;
import org.json.JSONObject;

public class Bot extends Module {

    public Bot() {
        super("robot");
    }

    public void init() {
        Server.begin(this);
    }

    public void teleop() {

    }

    public void autonomous() {

    }
}
