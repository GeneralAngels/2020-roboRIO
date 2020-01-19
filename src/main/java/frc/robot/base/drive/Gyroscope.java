package frc.robot.base.drive;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

// TODO go over with Idan or Alon to figure out what needs to go

public class Gyroscope {

    private static final int INITIAL_MEASUREMENTS = 200;
    private static final int INITIAL_MEASUREMENT_TIMEOUT = 20;
    private static final double ALPHA = 0.7;
    private static final double DEADZONE = 0.2;
    private static final double DELTA = 0.02;

    private PigeonIMU pigeon = new PigeonIMU(30);

    protected double bias = 0;
    protected double angle = 0;
    protected double previousX = 0, previousY = 0, previousZ = 0;
    protected double previousFiltered = 0;

    public Gyroscope() {
        super();
        recordShittyness();
    }

    protected long millis() {
        return (long) (Timer.getFPGATimestamp() * 1000);
    }

    protected void log(String string) {
        System.out.println(("Gyroscope: ") + string);
    }

    protected void recordShittyness() {
//        new Thread(() -> {
//            double summedShittyness = 0;
//            for (int m = 0; m < INITIAL_MEASUREMENTS; m++) {
//                try {
//                    summedShittyness += getRawGyroZ();
//                    Thread.sleep(INITIAL_MEASUREMENT_TIMEOUT);
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//            }
//            bias = summedShittyness / INITIAL_MEASUREMENTS;
//        }).start();
    }

    public double getBias() {
        return bias;
    }

    public double getLastFiltered() {
        return previousFiltered;
    }

    //    @Override
    public double getAngle() {
//        double raw = getRawGyroZ();
//        previousFiltered = filter(raw, ALPHA);
//        previousFiltered -= bias;
//        if (Math.abs(previousFiltered) > DEADZONE)
//            angle += previousFiltered * DELTA;
//        return angle;
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
//        pigeon.getRawGyro(ypr);
        return ypr[0] * (9.0 / 140.0);
    }

    protected double filter(double raw, double alpha) {
        double filtered = raw * (1 - alpha) + previousX * alpha;
        previousX = raw;
        return filtered;
    }
}