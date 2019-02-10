package frc.robot.efrat.systems.rgb;

import frc.robot.bobot.rgb.RGB;
import frc.robot.bobot.rgb.patterns.Rainbow;

import java.awt.*;

public class RobotIdle implements RGB.Pattern {

    private static final int MAX = 255;
    private static final int RAINBOW = 1;
    private static final int HATCH = 2;
    private static final int CARGO = 3;
    private static final int COLOR = 4;
    private static RobotIdle latest;
    private Rainbow rainbow = new Rainbow();
    private int state = RAINBOW;
    private boolean flag = false;
    private Color color;

    public RobotIdle() {
        latest = this;
    }

    public static RobotIdle getInstance() {
        return latest;
    }

    @Override
    public Color color(int length) {
        switch (state) {
            case RAINBOW:
                return rainbow.color(length);
            case HATCH:
                return flag ? Color.YELLOW : Color.BLACK;
            case CARGO:
                return flag ? Color.RED : Color.BLACK;
            case COLOR:
                return color;
        }
        return Color.BLACK;
    }

    @Override
    public void next(int ledCount) {
        rainbow.next(ledCount);
        flag = !flag;
    }

    public void needHatch() {
        state = HATCH;
    }

    public void needCargo() {
        state = CARGO;
    }

    public void rainbow() {
        state = RAINBOW;
    }

    public void cargoLoaded() {
        if (state == CARGO)
            rainbow();
    }

    public void hatchLoaded() {
        if (state == HATCH)
            rainbow();
    }

    public void color(Color color) {
        state = COLOR;
        this.color = color;
    }
}
