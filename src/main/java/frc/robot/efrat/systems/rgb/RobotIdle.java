package frc.robot.efrat.systems.rgb;

import frc.robot.bobot.rgb.RGB;
import frc.robot.bobot.rgb.patterns.Rainbow;
import frc.robot.bobot.rgb.patterns.Sinbow;

import java.awt.*;

public class RobotIdle implements RGB.Pattern {

    private static final int MAX = 255;
    private static RobotIdle latest;
    private LEDMode mode = LEDMode.Rainbow;
    private Rainbow rainbow = new Rainbow();
    private Sinbow sinbow = new Sinbow();
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
        switch (mode) {
            case Rainbow:
                return rainbow.color(length);
            case Sinbow:
                return sinbow.color(length);
            case Flash:
                return flag ? color : Color.BLACK;
            case Color:
                return color;
        }
        return Color.BLACK;
    }

    @Override
    public void next(int ledCount) {
        rainbow.next(ledCount);
        sinbow.next(ledCount);
        flag = !flag;
    }

    public void rainbow() {
        mode = LEDMode.Rainbow;
    }

    public void sinbow() {
        mode = LEDMode.Sinbow;
    }

    public void flash(Color color) {
        mode = LEDMode.Flash;
        this.color = color;
    }

    public void color(Color color) {
        mode = LEDMode.Color;
        this.color = color;
    }

    enum LEDMode {
        Color,
        Rainbow,
        Sinbow,
        Flash
    }
}
