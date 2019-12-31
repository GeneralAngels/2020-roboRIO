package frc.robot.base.rgb.patterns;

import frc.robot.base.rgb.RGB;

import java.awt.*;

public class TestPush implements RGB.Pattern {

    private boolean even = false;

    @Override
    public RGB.Packet next(int length) {

        even = !even;

        if (even)
            return RGB.Packet.colorPush(1, Color.RED);
        else
            return RGB.Packet.colorPush(1, Color.BLACK);

    }
}
