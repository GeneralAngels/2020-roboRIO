package frc.robot.kobi.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.base.control.PID;

public class KobiShooter extends frc.robot.base.Module {

    private static final double TALON_RATE = 100.0 / 1000.0; // 100ms/1s

    // Hood constants & things
    private static final double HOOD_THRESHOLD_DEGREES = 1;

    private static final double HOOD_MAXIMUM_POTENTIOMETER = 0.38; // TODO!!!!
    private static final double HOOD_MINIMUM_POTENTIOMETER = 0.22; // TODO!!!!
    private static final double HOOD_POTENTIOMETER_DELTA = (HOOD_MAXIMUM_POTENTIOMETER - HOOD_MINIMUM_POTENTIOMETER);

    private static final double HOOD_MAXIMUM_ANGLE = 64; // Verified by Libi (16/02/2020, Nadav)
    private static final double HOOD_MINIMUM_ANGLE = 35; // Verified by Libi (16/02/2020, Nadav)
    private static final double HOOD_ANGLE_DELTA = (HOOD_MAXIMUM_ANGLE - HOOD_MINIMUM_ANGLE);
    private static final double HOOD_COEFFICIENT = (HOOD_ANGLE_DELTA / HOOD_POTENTIOMETER_DELTA);

    private PID hoodPositionPID;

    private Servo hood;
    private Potentiometer potentiometer;

    private static double calculateAngle(double potentiometerPosition) {
        return HOOD_MAXIMUM_ANGLE - HOOD_COEFFICIENT * potentiometerPosition;
    }

    // Shooter things
    private static final double SHOOTER_ENCODER_TICKS = 4096;
    private static final double SHOOTER_WHEEL_RADIUS = 0.0762;

    private WPI_TalonSRX shooter1;
    private WPI_TalonSRX shooter2;
    private WPI_TalonSRX shooter3;
    private Encoder encoder;

    // Turret things
    private static final double TURRET_ENCODER_TICKS = 4096; // TODO find
    private static final double TURRET_THRESHOLD_TICKS = 10;
    private static final double TURRET_GEAR = 462.2; // Verified by Libi (16/02/2020, Nadav, Old = 182.6/17.5)

    private PID turretPositionPID;
    private WPI_TalonSRX turret;
    private int turretTargetPosition = 0;

    public KobiShooter() {
        super("shooter");

        // Hood things
        hoodPositionPID = new PID("hood_position_pid", 0, 0, 0, 0.001);
        hood = new Servo(9);
        potentiometer = new AnalogPotentiometer(0);

        // Turret things
        turretPositionPID = new PID("turret_position_pid", 0, 0, 0, 0.001);
        turret = new WPI_TalonSRX(19);
        turret.setSelectedSensorPosition(1);
        turret.configFactoryDefault();
        turret.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, 30);

        // Shooter things
        shooter1 = new WPI_TalonSRX(20);
        shooter2 = new WPI_TalonSRX(21);
        shooter3 = new WPI_TalonSRX(22);
        shooter1.setSelectedSensorPosition(1);
        shooter1.configFactoryDefault();
        shooter1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        shooter1.setSensorPhase(false); // Flip encoder polarity (+/-)
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
                String[] split = s.split(" ");
                boolean inFrame = Boolean.parseBoolean(split[0]);
                double speed = Double.parseDouble(split[1]);
                // Set position
                setTurretVelocity(speed);
                return new Tuple<>(true, "Moving turret");
            }
        });
    }

    public void updatePositions() {
        getShooterPosition();
        getTurretPosition();
        getHoodPosition();
    }

    // Sets the position using PID TODO check this!!! 16/02/2020
    public boolean setHoodPosition(double angle) {
        if (angle > HOOD_MINIMUM_ANGLE && angle < HOOD_MAXIMUM_ANGLE) {
            double measurement = potentiometer.get();
            double currentAngle = calculateAngle(measurement);
            hood.set(hoodPositionPID.PIDPosition(currentAngle, angle));
            return Math.abs(angle - currentAngle) < HOOD_THRESHOLD_DEGREES;
        }
        return false;
    }

    public void setShooterVelocity(double velocity) {
        // Velocity is M/S
        double input = (velocity * SHOOTER_ENCODER_TICKS * TALON_RATE) / (2 * Math.PI * SHOOTER_WHEEL_RADIUS);
        // Set is Tick/100ms
        shooter1.set(ControlMode.Velocity, input);
    }

    // Reset to reset the setpoint
    public boolean setTurretPosition(double delta, boolean reset) {
        // Reset setpoint
        if (reset)
            turretTargetPosition = getTurretPosition() + (int) (TURRET_ENCODER_TICKS * TURRET_GEAR * (delta / 360));
        // Calculate PID
        turret.set(turretPositionPID.PIDPosition(getTurretPosition(), turretTargetPosition));
        // Check threshold
        return Math.abs(turretTargetPosition - getTurretPosition()) < TURRET_THRESHOLD_TICKS;
    }

    public void setTurretVelocity(double speed) {
        turret.set(speed);
    }

    public int getShooterPosition() {
        int position = shooter1.getSelectedSensorPosition();
        set("shooter", String.valueOf(position));
        return position;
    }

    public int getTurretPosition() {
        int position = turret.getSelectedSensorPosition();
        set("turret", String.valueOf(position));
        return position;
    }

    public double getHoodPosition() {
        double position = potentiometer.get();
        set("hood", String.valueOf(position));
        return position;
    }
}
