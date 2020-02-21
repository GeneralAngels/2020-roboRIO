package frc.robot.kobi.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.base.Bot;
import frc.robot.kobi.Kobi;
import org.opencv.core.Mat;

public class KobiShooter extends frc.robot.base.Module {

    private static final double TALON_RATE = 100.0 / 1000.0; // 100ms/1s

    // Hood constants & things
    private static final double HOOD_THRESHOLD_DEGREES = 3;

    private static final double HOOD_MAXIMUM_POTENTIOMETER = 0.40; // Verified by Nadav
    private static final double HOOD_MINIMUM_POTENTIOMETER = 0.30; // Verified by Nadav
    private static final double HOOD_POTENTIOMETER_DELTA = (HOOD_MAXIMUM_POTENTIOMETER - HOOD_MINIMUM_POTENTIOMETER);

    public static final double HOOD_MAXIMUM_ANGLE = 64; // Verified by Libi (16/02/2020, Nadav)
    public static final double HOOD_MINIMUM_ANGLE = 35; // Verified by Libi (16/02/2020, Nadav)
    public static final double HOOD_SAFE_MAXIMUM_ANGLE = HOOD_MAXIMUM_ANGLE - 1; // Verified by Libi (16/02/2020, Nadav)
    public static final double HOOD_SAFE_MINIMUM_ANGLE = HOOD_MINIMUM_ANGLE + 1; // Verified by Libi (16/02/2020, Nadav)

    private static final double HOOD_ANGLE_DELTA = (HOOD_MAXIMUM_ANGLE - HOOD_MINIMUM_ANGLE);
    private static final double HOOD_COEFFICIENT = (HOOD_ANGLE_DELTA / HOOD_POTENTIOMETER_DELTA);

    private Servo hood;
    private Potentiometer potentiometer;

    private static double calculateAngle(double potentiometerPosition) {
        return HOOD_MAXIMUM_ANGLE - HOOD_COEFFICIENT * (potentiometerPosition - HOOD_MINIMUM_POTENTIOMETER);
    }

    // Shooter things
    private static final double SHOOTER_ENCODER_TICKS = 2048;
    private static final double SHOOTER_WHEEL_RADIUS = 0.0762;
    private static final double SHOOTER_VELOCITY_THRESHOLD = 2;

    private WPI_TalonSRX shooter1;
    private WPI_TalonSRX shooter2;
    private WPI_TalonSRX shooter3;

    // Turret things
    private static final double TURRET_ENCODER_TICKS = 4096; // Verified by Idan
    private static final double TURRET_THRESHOLD_TICKS = 10;
    private static final double TURRET_GEAR = 240.0 / 22.0; // Verified by Libi (16/02/2020, Nadav, Old = 182.6/17.5)

    private WPI_TalonSRX turret;
    private int turretOffsetPosition = 0;

    public KobiShooter() {
        super("shooter");

        // Hood things
        potentiometer = new AnalogPotentiometer(0);
        hood = new Servo(4);

        // Turret things
        turret = new WPI_TalonSRX(19);

        Kobi.setupMotor(turret, FeedbackDevice.PulseWidthEncodedPosition, 0, 0.0001, 0, 0.23);
        turret.setSensorPhase(true); // Flip encoder polarity (+/-)

        // Shooter things
        shooter1 = new WPI_TalonSRX(20);
        shooter2 = new WPI_TalonSRX(21);
        shooter3 = new WPI_TalonSRX(22);

        Kobi.setupMotor(shooter1, FeedbackDevice.CTRE_MagEncoder_Relative, 0.7, 0.0001, 1, 0.07); // OMG magic
        shooter1.setSensorPhase(false); // Flip encoder polarity (+/-)

        shooter1.setNeutralMode(NeutralMode.Coast);
        shooter2.setNeutralMode(NeutralMode.Coast);
        shooter3.setNeutralMode(NeutralMode.Coast);

        shooter1.setInverted(false);
        shooter2.setInverted(true);
        shooter3.setInverted(false);

        // Setup followers
        shooter2.follow(shooter1);
        shooter3.follow(shooter1);

        // Commands

        command("camera", new Command() {

            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                double speed = Double.parseDouble(s);
                // Set position
                setTurretVelocity(speed);
                return new Tuple<>(true, "Moving");
            }
        });

        command("turret", new Command() {

            private boolean reset = true;

            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                double delta = Double.parseDouble(s);
                if (reset) {
                    resetTurretPosition();
                    reset = false;
                } else {
                    reset = setTurretPosition(delta);
                }
                if (reset)
                    setTurretVelocity(0);
                return new Tuple<>(reset, "Moving");
            }
        });

        command("hood", new Command() {
            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                return new Tuple<>(setHoodPosition(Double.parseDouble(s)), "Moving");
            }
        });

        command("shooter", new Command() {
            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                double speed = Double.parseDouble(s); // m/s
                setShooterVelocity(speed);
                return new Tuple<>(true, "Speed set");
            }
        });
    }

    public void updatePositions() {
        getShooterPosition();
        getTurretPosition();
        getHoodPosition();
    }

    public boolean setHoodPosition(double angle) {
        if (angle >= HOOD_SAFE_MINIMUM_ANGLE && angle <= HOOD_SAFE_MAXIMUM_ANGLE) {
            double measurement = potentiometer.get();
            double currentAngle = calculateAngle(measurement);
            double error = deadband(angle - currentAngle, HOOD_THRESHOLD_DEGREES);
            double speed = 0;
            if (error != 0) {
                speed = error < 0 ? 1 : -1;
            }
            hood.set((speed + 1) / 2);
            return speed == 0;
        }
        return false;
    }

    public boolean setShooterVelocity(double targetVelocity) {
        if (targetVelocity != 0) {
            // Velocity is M/S
            double input = targetVelocity * ((SHOOTER_ENCODER_TICKS * TALON_RATE) / (2 * Math.PI * SHOOTER_WHEEL_RADIUS));
            // Set is Tick/100ms
            shooter1.set(ControlMode.Velocity, input);
//        log("A: " + shooter1.getMotorOutputPercent() + " B: " + shooter2.getMotorOutputPercent() + " C: " + shooter3.getMotorOutputPercent());
            double currentVelocity = shooter1.getSelectedSensorVelocity() / ((SHOOTER_ENCODER_TICKS * TALON_RATE) / (2 * Math.PI * SHOOTER_WHEEL_RADIUS));
            // Check threshold
            return Math.abs(targetVelocity - currentVelocity) < SHOOTER_VELOCITY_THRESHOLD;
        } else {
            shooter1.set(ControlMode.PercentOutput, 0);
            return true;
        }
    }

    public void resetTurretPosition() {
        turretOffsetPosition = getTurretPosition();
    }

    // Reset to reset the setpoint
    public boolean setTurretPosition(double angle) {
        // Calculate PID
        if (angle != 0) {
            double target = turretOffsetPosition + (int) (TURRET_ENCODER_TICKS * (angle / 360));
            double velocity = (target - getTurretPosition()) / (TALON_RATE);
            turret.set(ControlMode.Velocity, velocity);
            // Check threshold
            return Math.abs(target - getTurretPosition()) < TURRET_THRESHOLD_TICKS;
        }
        return true;
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

    private double deadband(double value, double threshold) {
        if (Math.abs(value) < threshold)
            return 0;
        return value;
    }
}
