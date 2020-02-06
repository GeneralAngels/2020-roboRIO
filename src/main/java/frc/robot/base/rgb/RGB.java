package frc.robot.base.rgb;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.base.Module;

import java.awt.*;
import java.util.*;

/**
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

public class RGB extends Module {

    private static final int TOO_FAST_MILLIS = 20;

    private long updateMillis = 0;

    private Mode sentMode = null;
    private Mode mode = Mode.Fill;

    private Color sentColor = null;
    private Color color = Color.BLACK;

    private SerialPort serial;

    public RGB() {
        super("rgb");
        try {
            this.serial = new SerialPort(9600, SerialPort.Port.kUSB);
        } catch (Exception ignored) {
            this.serial = null;
            log("RGB serial Initialization failure");
        }
        command("color", new Command() {
            @Override
            public String execute(String s) throws Exception {
                String[] split = s.split(" ");
                if (!(split.length == 3))
                    return "Must have 3 parameters";
                // Parse things
                int r = Integer.parseInt(split[0]);
                int g = Integer.parseInt(split[1]);
                int b = Integer.parseInt(split[2]);
                // Validate
                if (!(r < 256 && g < 256 && b < 256))
                    return "Not in valid range";
                // Set the color
                setColor(new Color(r, g, b));
                // Return OK
                return "OK";
            }
        });
        command("mode", new Command() {
            @Override
            public String execute(String s) throws Exception {
                if (s.equals("fill")){
                    setMode(Mode.Fill);
                    return "OK, Fill";
                }else if (s.equals("slide")){
                    setMode(Mode.Slide);
                    return "OK, Slide";
                }
                // Return help
                return "Must be 'fill' or 'slide'";
            }
        });
    }

    public void setMode(Mode mode) {
        this.mode = mode;
        doUpdate();
    }

    public void setColor(Color color) {
        this.color = color;
        doUpdate();
    }

    private void doUpdate() {
        // if (sentColor != color || sentMode != mode) {
        if (millis() > updateMillis + TOO_FAST_MILLIS) {
            updateMillis = millis();
            sentMode = mode;
            sentColor = color;
            if (this.serial != null) {
                serial.write(new byte[]{
                        (byte) (mode == Mode.Fill ? 0 : 1),
                        (byte) ((sentColor.getRed() / 8) + 2),
                        (byte) ((sentColor.getGreen() / 8) + 2),
                        (byte) ((sentColor.getBlue() / 8) + 2),
                        (byte) (mode == Mode.Fill ? 0 : 1),
                }, 5);
            }
        }
        // }
    }

    public enum Mode {
        Slide,
        Fill
    }
}
