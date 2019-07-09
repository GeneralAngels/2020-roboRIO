package frc.robot.efrat.systems.rgb;

import frc.robot.base.rgb.RGB;

import java.awt.*;

public class Idling implements RGB.Pattern {
    private final static int BLINK_SIZE = 4;
    private Color color = Color.MAGENTA;
    private boolean paint = true;
    private int height = 0;
    private int loop = 0;

    @Override
    public RGB.Fill fill(int length) {
//        System.out.println(height);
        int target = length / 2 + (length % 2);
        if (paint) {
            if (loop % 2 == 0) height++;
            if (height % target == 0) paint = false;
            return new RGB.Fill(color, (height % target) + 1, loop % 2 == 0);
        } else {
            paint = true;
            return new RGB.Fill(Color.BLACK, length, false);
        }
    }

    public void shiri(boolean state) {
        loop = 0;
        paint = false;
        if (state) {
            color = Color.MAGENTA;
        } else {
            color = Color.GREEN;
        }
    }

    @Override
    public void next(int length) {
        loop++;
    }
}
