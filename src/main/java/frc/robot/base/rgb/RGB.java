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

    private static final int REFRESH_RATE = 10;

    private Mode sentMode = null;
    private Mode mode = Mode.Fill;

    private Color sentColor = null;
    private Color color = Color.BLACK;

    private SerialPort serial;

    private Timer timer;

    public RGB() {
        super("rgb");
        try {
            this.serial = new SerialPort(9600, SerialPort.Port.kUSB);
        } catch (Exception ignored) {
            this.serial = null;
            log("RGB serial Initialization failure");
        }
        timer = new Timer();
        timer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                loop();
            }
        }, 0, 1000 / REFRESH_RATE);
        command("pattern", new Command() {
            @Override
            public String execute(String s) throws Exception {

                return "OK";
            }
        });
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public void setColor(Color color) {
        this.color = color;
    }

    private void loop() {
        if (sentColor != color || sentMode != mode) {
            sentMode = mode;
            sentColor = color;
            if (this.serial != null) {
                try {
                    serial.write(new byte[]{
                            (byte) (mode == Mode.Fill ? 0 : 1),
                            (byte) ((sentColor.getRed() / 8) + 2),
                            (byte) ((sentColor.getGreen() / 8) + 2),
                            (byte) ((sentColor.getBlue() / 8) + 2)
                    }, 4);
                } catch (Exception ignored) {
                    log("RGB serial transmit failure");
                }
            }
        }
    }

    enum Mode {
        Slide,
        Fill
    }
}
