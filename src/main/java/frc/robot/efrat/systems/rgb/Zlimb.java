package frc.robot.efrat.systems.rgb;

import frc.robot.base.rgb.RGB;

import java.awt.*;

public class Zlimb implements RGB.Pattern {

    private int height = 0;
    private int loop = 0;
    private boolean paint = true;

    @Override
    public RGB.Fill fill(int length) {
//        System.out.println(height);
        int target = length / 2 + (length % 2);
        if (paint) {
            if (loop % 2 == 0) height++;
            if (height % target == 0) paint = false;
            return new RGB.Fill(Color.CYAN, (height % target) + 1, loop % 2 == 0);
        } else {
            paint = true;
            return new RGB.Fill(Color.BLACK, length, false);
        }
    }

    @Override
    public void next(int length) {
        loop++;
    }
}
