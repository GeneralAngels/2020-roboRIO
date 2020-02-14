package frc.robot.kobi.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.base.control.PID;

public class KobiShooter extends frc.robot.base.Module {

    private static final double ENCODER_TICKS = 4096;
    private static final double TURRET_TICKS_TOLERANCE = 50; // 50 tick tolerance
    private static final double SHOOTER_WHEEL_RADIUS = 0.0762; // M

    private static final double TALON_RATE = 100.0 / 1000.0; // 100ms/1s

    private WPI_TalonSRX shooter1;
    private WPI_TalonSRX shooter2;
    private WPI_TalonSRX shooter3;

    private WPI_TalonSRX turret;
    private Servo hood;

    public KobiShooter() {
        super("shooter");

        turret = new WPI_TalonSRX(19);

        shooter1 = new WPI_TalonSRX(20);
        shooter2 = new WPI_TalonSRX(21);
        shooter3 = new WPI_TalonSRX(22);

        hood = new Servo(9);

        shooter1.setSelectedSensorPosition(1);
        turret.setSelectedSensorPosition(1);

        shooter1.configFactoryDefault();
        turret.configFactoryDefault();

        shooter1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        turret.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, 30);

        shooter1.setSensorPhase(false); // Flip encoder polarity (+/-)
        // turret.setSensorPhase(true); todo

        shooter1.configNominalOutputForward(0, 30);
        shooter1.configNominalOutputReverse(0, 30);
        shooter1.configPeakOutputForward(1, 30);
        shooter1.configPeakOutputReverse(-1, 30);

        shooter1.config_kP(0, 0, 30);
        shooter1.config_kI(0, 0.00003, 30);
        shooter1.config_kD(0, 0, 30);
        shooter1.config_kF(0, 0.04, 30);

        shooter3.setInverted(true);

        shooter2.follow(shooter1);
        shooter3.follow(shooter1);

        command("turret", new Command() {

            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                double speed = Double.parseDouble(s);
                // Set position
                setTurretVelocity(speed);
                return new Tuple<>(true, "Moving turret");
            }
        });
    }

    public void setShooterVelocity(double velocity) {
        // Velocity is M/S
        double input = (velocity * ENCODER_TICKS * TALON_RATE) / (2 * Math.PI * SHOOTER_WHEEL_RADIUS);
        // Set is Tick/100ms
        shooter1.set(ControlMode.Velocity, input);
    }

    public void setTurretVelocity(double speed) {
        turret.set(speed);
    }

    public void setHoodPosition(double position) {
        hood.set(position);
    }
}
