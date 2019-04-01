package frc.robot.bobot.rgb;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.bobot.Subsystem;

import java.awt.*;
import java.util.Timer;
import java.util.TimerTask;

public class RGB extends Subsystem {
    // Already OCed To 75Hz. Dont overclock higher!
    // Base clock: 20Hz
    private static final int REFRESH_RATE = 20;
    private int length = 1, divider = 1;
    private Timer timer;
    private SerialPort serial;
    private Pattern pattern;

    public RGB(int length, int divider) {
        if (length > 0)
            this.length = length;
        if (divider > 0)
            this.divider = divider;
        try {
            this.serial = new SerialPort(9600, SerialPort.Port.kUSB);
        } catch (Exception ignored) {
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
            send(pattern.fill(length));
            pattern.next(length);
        }
    }

    private void send(Fill fill) {
        try {
            byte[] packet = new byte[5];
            // RGB
//            packet[0] = (byte) (color.getRed() / divider);
//            packet[1] = (byte) (color.getGreen() / divider);
//            packet[2] = (byte) (color.getBlue() / divider);
            // BGR
            if (fill != null) {
                packet[0] = (byte) (fill.direction?1:0);
                packet[1] = (byte) (fill.length);
                packet[2] = (byte) (fill.color.getBlue() / divider);
                packet[3] = (byte) (fill.color.getGreen() / divider);
                packet[4] = (byte) (fill.color.getRed() / divider);
            }
            serial.write(packet, 5);
        } catch (Exception ignored) {

        }
    }

    public interface Pattern {
        Fill fill(int length);

        void next(int length);
    }

    public static class Fill {
        private Color color;
        private boolean direction = false;
        private int length = 0;

        public Fill(Color color, int length, boolean direction) {
            this.color = color;
            this.length = length;
            this.direction = direction;
        }

        public Fill(Color color) {
            this.color = color;
        }
    }
}
