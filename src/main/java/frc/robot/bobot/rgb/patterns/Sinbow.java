package frc.robot.bobot.rgb.patterns;

import frc.robot.bobot.rgb.RGB;

import java.awt.*;

public class Sinbow implements RGB.Pattern {

    private static final int MAX = 255;
    private int current = 0;

    @Override
    public Color color(int length) {
        int r = 0, g = 0, b = 0;
        r = (int) (Math.sin(current) * MAX + MAX / 2);
        g = (int) (Math.sin(current + 45) * MAX + MAX / 2);
        b = (int) (Math.sin(current + 90) * MAX + MAX / 2);
        return new Color(r, g, b);
    }

    @Override
    public void next(int ledCount) {
        current += 1;
    }
}
