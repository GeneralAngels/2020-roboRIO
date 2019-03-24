package frc.robot.efrat.systems.rgb;

import frc.robot.bobot.rgb.RGB;

import java.awt.*;

public class Speedy implements RGB.Pattern {
    private Color color = Color.MAGENTA;
    private boolean paint = true;
    private double height = 0;
    private int loop = 0;

    @Override
    public RGB.Fill fill(int length) {
//        System.out.println(height);
        int target = length / 2 + (length % 2);
        int realHeight = (int) (target * height);
//            return new RGB.Fill(color, (height % target) + 1, loop % 2 == 0);
        return new RGB.Fill(Color.BLACK, length, false);
    }

    public void state(boolean state) {
        loop = 0;
        paint = false;
        if (state) {
            color = Color.MAGENTA;
        } else {
            color = new Color(255, 200, 0);
        }
    }

    public void idle(double height) {
        this.height = height;
    }

    @Override
    public void next(int length) {
        loop++;
    }
}
