package frc.robot.base.rgb;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.base.Module;

import java.awt.*;
import java.util.Timer;
import java.util.TimerTask;

public class RGB extends Module {

    private static final int REFRESH_RATE = 10;
    private int length = 1;
    private SerialPort serial;
    private Pattern pattern;
    private Timer timer;

    public RGB(int length, int divider) {
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

        private byte[] packet = new byte[8];

        public Packet(boolean direction, int command, int p1, int p2, int r, int g, int b) {
            init(direction, command, p1, p2, r, g, b);
        }

        public Packet(boolean direction, int command, int p1, int p2, Color color) {
            init(direction, command, p1, p2, color.getRed(), color.getGreen(), color.getBlue());
        }

        private void init(boolean direction, int command, int p1, int p2, int r, int g, int b) {
            packet[0] = (byte) 0xFF;
            packet[1] = (byte) (direction ? 1 : 0);
            packet[2] = (byte) command;
            packet[3] = (byte) p1;
            packet[4] = (byte) p2;
            packet[5] = (byte) (r / BRIGHTNESS_DIVISOR);
            packet[6] = (byte) (g / BRIGHTNESS_DIVISOR);
            packet[7] = (byte) (b / BRIGHTNESS_DIVISOR);
        }

        public byte[] get() {
            return packet;
        }
    }
}
