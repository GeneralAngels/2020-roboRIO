package frc.robot.bobot.rgb.patterns;

import frc.robot.bobot.rgb.RGB;

import java.awt.*;

public class Sinbow implements RGB.Pattern {

    private static final int MAX = 255;
    private long current = 0;
    private int sineOffset = 60;

    @Override
    public Color color(int length) {
        int r = 0, g = 0, b = 0;
        long c=current;
        r = (int) ((Math.sin(Math.toRadians(c)) / 2.0 + 0.5) * MAX);
        g = (int) ((Math.sin(Math.toRadians(c + sineOffset)) / 2.0 + 0.5) * MAX);
        b = (int) ((Math.sin(Math.toRadians(c + 2 * sineOffset)) / 2.0 + 0.5) * MAX);
        return new Color(r, g, b);
    }

    @Override
    public void next(int ledCount) {
        current += 1;
    }
}
