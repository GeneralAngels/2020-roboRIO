package frc.robot.base.rgb.patterns;

import frc.robot.base.rgb.RGB;

import java.awt.*;

public class Rainbow implements RGB.Pattern {

    private static final int MAX = 255;
    private int current = 0;

    @Override
    public RGB.Packet packet(int length) {
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
        return new RGB.Packet(true, RGB.Packet.PUSH,0,0,r,g,b);
    }

    @Override
    public void next(int ledCount) {
        current += 1;
    }
}
