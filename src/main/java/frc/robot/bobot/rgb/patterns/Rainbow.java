package frc.robot.bobot.rgb.patterns;

import frc.robot.bobot.rgb.RGB;

import java.awt.*;

public class Rainbow implements RGB.Pattern {

    private static final int MAX = 255;
    private int current = 0;

    @Override
    public Color color(int length) {
        int functionX = current % ((int) (1.5 * (double) MAX));
        int ranger = (MAX / 2) + 1;
        int rising = (2 * (functionX % ranger)), falling = (MAX - rising);
        int r = 0, g = 0, b = 0;
        if (functionX >= 0) {
            r = rising;
            g = 0;
            b = falling;
        }
        if (functionX > MAX / 2) {
            r = falling;
            g = rising;
            b = 0;
        }
        if (functionX > MAX) {
            r = 0;
            g = falling;
            b = rising;
        }
        return new Color(r, g, b);
    }

    @Override
    public void next(int ledCount) {
        current += 1;
    }
}
