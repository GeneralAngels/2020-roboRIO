package frc.robot.base.drive;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

public abstract class Gyroscope {

    private static PigeonIMU pigeon = new PigeonIMU(30);

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

    public static void setAngle(double angle) {
        pigeon.setYaw(angle);
    }

    public static void reset() {
        pigeon.setYaw(0);
        pigeon.setFusedHeading(0);
//        pigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.BootTareGyroAccel);
    }
}