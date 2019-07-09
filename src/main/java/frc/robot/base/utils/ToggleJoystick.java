package frc.robot.base.utils;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.base.Module;

/**
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

// This class takes a joystick and maps it to many Toggle classes.
// Gives us the ability to use all of the physical-buttons as toggles or triggers.

public class ToggleJoystick extends Module {

    private Joystick actual;
    private Toggle trigger, sidebutton;

    public ToggleJoystick(Joystick actual) {
        this.actual = actual;
    }

    public void update() {
        trigger.update(actual.getTrigger());
    }
}
