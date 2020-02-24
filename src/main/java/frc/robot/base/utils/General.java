package frc.robot.base.utils;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class General {

    public static void setupMotor(WPI_TalonSRX talon, FeedbackDevice feedbackDevice, double kP, double kI, double kD, double kF) {
        talon.setSelectedSensorPosition(0);
        talon.configFactoryDefault();
        talon.configSelectedFeedbackSensor(feedbackDevice, 0, 30);
        talon.configNominalOutputForward(0, 30);
        talon.configNominalOutputReverse(0, 30);
        talon.configPeakOutputForward(1, 30);
        talon.configPeakOutputReverse(-1, 30);
        talon.config_kP(0, kP, 30);
        talon.config_kI(0, kI, 30);
        talon.config_kD(0, kD, 30);
        talon.config_kF(0, kF, 30);
    }

    public static double compassify(double angle) {
        angle %= 360;
        if (Math.abs(angle) > 180) {
            angle += (angle > 0) ? -360 : 360;
        }
        return angle;
    }

    public static double deadband(double value, double tolerance) {
        if (Math.abs(value) < tolerance)
            return 0;
        return value;
    }
}
