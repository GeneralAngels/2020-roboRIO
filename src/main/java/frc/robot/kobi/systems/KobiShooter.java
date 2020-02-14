package frc.robot.kobi.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.base.control.PID;

public class KobiShooter extends frc.robot.base.Module {

    private static final double TIMEOUT = 50;

    private static final double GEAR = 1.0 / 3.0;
    private static final double ENCODER_TICKS = 4096;
    private static final double TALON_VELOCITY_RATE = 1000.0 / 100.0; // 10Hz

    private WPI_TalonSRX motor1;
    private WPI_TalonSRX motor2;
    private WPI_TalonSRX motor3;

    private WPI_TalonSRX turret;
    private Servo hood;

    private PID motorsControlVelocity;

    // Encoder
    private int encoder = 0;
    private int previousEncoder = 0;

    private double previousTime = 0;

    public KobiShooter() {
        super("shooter");

//        motor1 = new WPI_TalonSRX(20);
//        motor2 = new WPI_TalonSRX(21);
//        motor3 = new WPI_TalonSRX(22);

        turret = new WPI_TalonSRX(13);
//
//        hood = new Servo(9);
//
//        motor1.setSelectedSensorPosition(1);
//
//        motor1.configFactoryDefault();
//        motor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
//        motor1.setSensorPhase(true);
//        motor1.configNominalOutputForward(0, 30);
//        motor1.configNominalOutputReverse(0, 30);
//        motor1.configPeakOutputForward(1, 30);
//        motor1.configPeakOutputReverse(-1, 30);
//        motor1.config_kP(0, 0, 30);
//        motor1.config_kI(0, 0, 30);
//        motor1.config_kD(0, 0, 30);
//        motor1.config_kF(0, 0.7, 30);
//
//        motor3.setInverted(true);
//
//        motor2.follow(motor1);
//        motor3.follow(motor1);
//        motorsControlVelocity = new PID("motorControlVelocity", 0.005, 0.007, 0.05, 0.05); // todo not finished. pay attention! - very small k's!
    }

    public void setVelocity(double velocity) {
        // Velocity is RPM
        double input = velocity * GEAR * (ENCODER_TICKS / (60 * TALON_VELOCITY_RATE));
        // Input is (?)
        motor1.set(ControlMode.Velocity, input);
    }

    public void setTurretPosition(double position) {
        // Position is degrees
        double input = (position / 360.0) * GEAR * ENCODER_TICKS;
        turret.set(ControlMode.Position, input);
    }

    public void setHoodPosition(double position) {
        hood.set(position);
    }

    public int getPosition() {
        set("encoder", String.valueOf(this.encoder));
        previousEncoder = encoder;
        previousTime = millis();
        if ((encoder = motor1.getSelectedSensorPosition()) == 0)
            encoder = previousEncoder;
        return encoder;
    }
}
