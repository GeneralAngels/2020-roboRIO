package frc.robot.efrat.systems.rgb;

import frc.robot.bobot.rgb.RGB;

import java.awt.*;

public class Idling implements RGB.Pattern {
    private final static int BLINK_SIZE = 4;
    private Color color = Color.MAGENTA;
    private int height = 0;
    private int loop = 0;

    @Override
    public RGB.Fill fill(int length) {
//        System.out.println(height);
        int target = length / 2 + (length % 2);
        if (loop % 4 < 2) {
            if (loop % 2 == 0) height++;
            return new RGB.Fill(color, (height % target) + 1, loop % 2 == 0);
        } else {
            if ((height % target) - BLINK_SIZE + 1 > 0)
                return new RGB.Fill(Color.BLACK, (height % target) - BLINK_SIZE + 1, loop % 2 == 0);
        }
        return null;
    }

    public void shiri(boolean state) {
        loop = 0;
        if (state) {
            color = Color.MAGENTA;
        } else {
            color = new Color(255, 100, 0);
        }
    }

    @Override
    public void next(int length) {
        loop++;
    }
}
