package frc.robot.efrat.systems.rgb;

import frc.robot.bobot.rgb.RGB;
import frc.robot.bobot.rgb.patterns.Rainbow;

import java.awt.*;

public class RobotIdle implements RGB.Pattern {

    private static final int MAX = 255;
    private static RobotIdle latest;
    private long loop = 0;
    private LEDMode mode = LEDMode.Rainbow;
    private Rainbow rainbow = new Rainbow();
    private Zlimb climb = new Zlimb();
    private Color color;

    public RobotIdle() {
        latest = this;
    }

    public static RobotIdle getInstance() {
        return latest;
    }

    @Override
    public RGB.Fill fill(int length) {
        switch (mode) {
            case Climb:
                return climb.fill(length);
            case Rainbow:
                return rainbow.fill(length);
            case Flash:
                return loop % 5 == 0 ? new RGB.Fill(color, length, false) : new RGB.Fill(Color.BLACK);
            case Color:
                return new RGB.Fill(color, length, false);
        }
        return new RGB.Fill(Color.BLACK, length, false);
    }

    @Override
    public void next(int ledCount) {
        rainbow.next(ledCount);
        climb.next(ledCount);
        loop++;
    }

    public void climb() {
        mode = LEDMode.Climb;
    }

    public void rainbow() {
        mode = LEDMode.Rainbow;
    }

    public void flash(Color color) {
        mode = LEDMode.Flash;
        this.color = color;
    }

    public void color(Color color) {
        mode = LEDMode.Color;
        this.color = color;
    }

    public void idle() {
        rainbow();
    }

    enum LEDMode {
        Color,
        Rainbow,
        Flash,
        Climb
    }
}
