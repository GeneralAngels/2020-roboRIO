package frc.robot.base.utils;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.base.Module;

/**
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

public class ToggleJoystick extends Module {

    private Joystick actual;
    private Toggle trigger, sidebutton;

    public ToggleJoystick(Joystick actual){
        this.actual = actual;
    }

    public void update(){
        trigger.update(actual.getTrigger());
    }
}
