package frc.robot.base.rgb;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.base.Module;
import frc.robot.base.rgb.patterns.TestPush;

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
    private Pattern pattern = new TestPush();
    private Timer timer;

    public RGB(int length) {
        super("rgb");
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
        addCommand("pattern", new Command() {
            @Override
            public String execute(String s) throws Exception {

                return "OK";
            }
        });
    }

    public void setPattern(Pattern pattern) {
        this.pattern = pattern;
    }

    private void loop() {
        if (pattern != null) {
            send(pattern.next(length));
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
        Packet next(int length);
    }

    public static class Packet {

        private byte[] packet = new byte[5];

        private Packet() {

        }

        public static Packet colorOverwrite(int amount, Color color) {
            Packet packet = new Packet();
            packet.packet[0] = 2;
            packet.packet[1] = (byte) amount;
            packet.packet[2] = (byte) color.getRed();
            packet.packet[3] = (byte) color.getGreen();
            packet.packet[4] = (byte) color.getBlue();
            return packet;
        }

        public static Packet colorPush(int amount, Color color) {
            Packet packet = new Packet();
            packet.packet[0] = 1;
            packet.packet[1] = (byte) amount;
            packet.packet[2] = (byte) color.getRed();
            packet.packet[3] = (byte) color.getGreen();
            packet.packet[4] = (byte) color.getBlue();
            return packet;
        }

        public static Packet clear() {
            Packet packet = new Packet();
            packet.packet[0] = 0;
            packet.packet[1] = 0;
            packet.packet[2] = 0;
            packet.packet[3] = 0;
            packet.packet[4] = 0;
            return packet;
        }

        public static Packet doSync() {
            Packet packet = new Packet();
            for (int i = 0; i < packet.packet.length; i++)
                packet.packet[i] = 0;
            return packet;
        }

        public byte[] get() {
            return packet;
        }
    }
}
