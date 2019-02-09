package frc.robot.bobot.drive;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;

public class Gyroscope extends ADXRS450_Gyro {
    public double offset;
    public double countedAngle = 0;
    public double tolerance = 0.5;
    public double angular_velocity = 0;
    public long time = millis();

    protected long millis() {
        return (long) (Timer.getFPGATimestamp() * 1000);
    }

    protected void log(String string) {
        System.out.println(("Gyroscope: ") + string);
    }

    public void calculateRate() {
        float avg = 0;
        for (int i = 0; 200 > i; i++) {
            avg += getRate();
        }
        offset = avg / 200;
    }

    @Override
    public void calibrate() {
        time = millis();
        super.calibrate();
    }

    public void resetAngle() {
        countedAngle = 0;
    }

    public double getCountedAngle() {
        double delta = 0.02;
        long currentTime = millis();
        if (time - currentTime > 0)
            delta = 1.0 / (time - currentTime);
        time = currentTime;
        angular_velocity = getRate() - offset;
//        log(Double.toString(angle));
        if (Math.abs(angular_velocity) > tolerance) {

            countedAngle += angular_velocity * delta;
        }
//        log("delta: " + Double.toString(delta)+",offset: " + Double.toString(offset) + ",getRate: " + Double.toString(getRate()) + ",vel: " +  Double.toString(angular_velocity ));
//        log("get_angle: " + Double.toString(getAngle()) + ",get_counted_angle: " + countedAngle);
        return countedAngle;
    }
}