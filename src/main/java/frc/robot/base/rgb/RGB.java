package frc.robot.base.rgb;

import com.ga2230.shleam.advanced.frc.FRCModule;
import com.ga2230.shleam.base.structure.Function;
import com.ga2230.shleam.base.structure.Result;
import edu.wpi.first.wpilibj.SerialPort;

import java.awt.*;

/**
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

public class RGB extends FRCModule {

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
        } catch (Exception exception) {
            this.serial = null;
            log("RGB serial Initialization failure: " + exception.toString());
        }

        register("color", new Function() {
            @Override
            public Result execute(String parameter) throws Exception {
                String[] split = parameter.split(" ");
                // Parse things
                int r = Integer.parseInt(split[0]);
                int g = Integer.parseInt(split[1]);
                int b = Integer.parseInt(split[2]);
                // Set the color
                setColor(new Color(r, g, b));
                // Return OK
                return Result.finished("Set");
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
    }

    public enum Mode {
        Slide,
        Fill
    }
}
