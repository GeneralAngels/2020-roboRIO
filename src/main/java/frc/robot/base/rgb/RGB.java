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
    private int length = 1;
    private SerialPort serial;
    private Pattern pattern;
    private Timer timer;

    public RGB(int length) {
        if (length > 0)
            this.length = length;
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
    }

    public void setPattern(Pattern pattern) {
        this.pattern = pattern;
    }

    private void loop() {
        if (pattern != null) {
            send(pattern.packet(length));
            pattern.next(length);
        }
    }

    private void send(Packet packet) {
        if (this.serial != null) {
            try {
                serial.write(packet.get(), packet.get().length);
            } catch (Exception ignored) {
                log("RGB serial transmit failure");
            }
        }
    }

    public interface Pattern {
        Packet packet(int length);

        void next(int length);
    }

    public static class Packet {

        public static final int BRIGHTNESS_DIVISOR = 1;

        public static final int FILL = 0;
        public static final int PUSH = 1;

        private ArrayList<Byte> packet = new ArrayList<>();

        public Packet(int command, int r, int g, int b) {
            init(command, r, g, b);
        }

        public Packet(int command, Color color) {
            init(command, color.getRed(), color.getGreen(), color.getBlue());
        }

        private void init(int command, int r, int g, int b) {
            packet.add((byte) command);
            packet.add((byte) (r / BRIGHTNESS_DIVISOR));
            packet.add((byte) (g / BRIGHTNESS_DIVISOR));
            packet.add((byte) (b / BRIGHTNESS_DIVISOR));
        }

        public void addParameter(Byte parameter) {
            packet.add(parameter);
        }

        public byte[] get() {
            byte[] compiled = new byte[packet.size()];
            for (int i = 0; i < compiled.length; i++) {
                compiled[i] = packet.get(i);
            }
            return compiled;
        }
    }
}
