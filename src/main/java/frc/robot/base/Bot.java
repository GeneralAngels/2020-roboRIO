package frc.robot.base;

import frc.robot.base.communications.Communicator;
import org.json.JSONObject;

/**
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

public class Bot extends Module {
    private Communicator communicator = null;

    public void init() {
        communicator = new Communicator(this);
    }

    public void teleop() {
        if (communicator != null) communicator.update();
    }

    public void autonomous() {
        if (communicator != null) communicator.update();
    }
}
