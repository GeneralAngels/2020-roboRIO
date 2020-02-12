package frc.robot.base.drive;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

public class Gyroscope {

    private static PigeonIMU pigeon = new PigeonIMU(30);

    private static long millis() {
        return (long) (Timer.getFPGATimestamp() * 1000);
    }

    private static void log(String string) {
        System.out.println(("Gyroscope: ") + string);
    }

    public static double getAngle() {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr[0];
    }

    public static double getAngularVelocity() {
        double[] xyz = new double[3];
        pigeon.getRawGyro(xyz);
        return xyz[2];
    }

    public static void reset() {
        pigeon.setYaw(0);
        pigeon.setFusedHeading(0);
//        pigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.BootTareGyroAccel);
    }
}