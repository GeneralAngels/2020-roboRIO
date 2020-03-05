package frc.robot.kobi.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ga2230.shleam.advanced.frc.FRCModule;
import com.ga2230.shleam.base.structure.Function;
import com.ga2230.shleam.base.structure.Result;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.base.utils.General;

public class KobiShooter extends FRCModule {

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
    private static final double TURRET_THRESHOLD_VELOCITY = 0.1;
    private static final double TURRET_THRESHOLD_TICKS = 10;
    private static final double TURRET_GEAR = 240.0 / 22.0; // Verified by Libi (16/02/2020, Nadav, Old = 182.6/17.5)

    private int turretOffsetTicks;

    private WPI_TalonSRX turret;

    // Setpoints
    private double shooterVelocitySetPoint, hoodPositionSetPoint, turretVelocitySetPoint;
    private boolean setpointLock = false;

    public KobiShooter() {
        super("shooter");

        // Hood things
        potentiometer = new AnalogPotentiometer(0);
        hood = new Servo(4);

        // Turret things
        turret = new WPI_TalonSRX(19);

        General.setupMotor(turret, FeedbackDevice.PulseWidthEncodedPosition, 0, 0.0001, 0, 0.23);
        turret.setSensorPhase(true); // Flip encoder polarity (+/-)

        // Shooter things
        shooter1 = new WPI_TalonSRX(20);
        shooter2 = new WPI_TalonSRX(21);
        shooter3 = new WPI_TalonSRX(22);

        General.setupMotor(shooter1, FeedbackDevice.CTRE_MagEncoder_Relative, 1, 0.00001, 0, 0.07); // OMG magic
        shooter1.setSensorPhase(true); // Flip encoder polarity (+/-)

        shooter1.setNeutralMode(NeutralMode.Coast);
        shooter2.setNeutralMode(NeutralMode.Coast);
        shooter3.setNeutralMode(NeutralMode.Coast);

        shooter1.setInverted(true);
        shooter2.setInverted(true);
        shooter3.setInverted(false);

        // Setup followers
        shooter2.follow(shooter1);
        shooter3.follow(shooter1);

        // Commands

        register("setpoints", new Function() {
            @Override
            public Result execute(String parameter) throws Exception {
                if (!setpointLock) {
                    String[] parameters = parameter.split(" ");
                    shooterVelocitySetPoint = Double.parseDouble(parameters[0]);
                    hoodPositionSetPoint = Double.parseDouble(parameters[1]);
                    turretVelocitySetPoint = Double.parseDouble(parameters[2]);
                    set("shooter-setpoint", parameters[0]);
                    set("hood-setpoint", parameters[1]);
                    set("turret-setpoint", parameters[2]);
                    log("SetPoints: " + shooterVelocitySetPoint + " " + hoodPositionSetPoint + " " + turretVelocitySetPoint);
                    return Result.finished("Thanks man");
                } else {
                    return Result.notFinished("Setpoint lock");
                }
            }
        });

        register("follow", new Function() {
            @Override
            public Result execute(String parameter) throws Exception {
                if (parameter.equals("turret")) {
                    return Result.create(followTurretSetPoint(), "Following on turret");
                } else if (parameter.equals("shooter")) {
                    return Result.create(followShooterSetPoint(), "Following on shooter");
                } else if (parameter.equals("hood")) {
                    return Result.create(followHoodSetPoint(), "Following on hood");
                }
                return Result.notFinished("Follow what");
            }
        });
    }

    public void setSetPointLock(boolean setpointLock) {
        this.setpointLock = setpointLock;
    }

    public void updatePositions() {
        getShooterPosition();
        getTurretPosition();
        getHoodPosition();
    }

    public double getHoodSetPoint() {
        return hoodPositionSetPoint;
    }

    public double getShooterSetPoint() {
        return shooterVelocitySetPoint;
    }

    public double getTurretSetPoint() {
        return turretVelocitySetPoint;
    }

    public boolean followHoodSetPoint() {
        return setHoodPosition(hoodPositionSetPoint);
    }

    public boolean followShooterSetPoint() {
        return setShooterVelocity(shooterVelocitySetPoint);
    }

    public boolean followTurretSetPoint() {
        setTurretVelocity(-turretVelocitySetPoint / 5);
        return General.deadband(-turretVelocitySetPoint / 5, 0.05) == 0;
    }

    public void resetTurretPosition() {
        this.turretOffsetTicks = turret.getSelectedSensorPosition();
    }

    public boolean setHoodPosition(double angle) {
        if (angle >= HOOD_SAFE_MINIMUM_ANGLE && angle <= HOOD_SAFE_MAXIMUM_ANGLE) {
            // Fetch measurements
            double measurement = potentiometer.get();
            double currentAngle = calculateAngle(measurement);
            // Calculate error
            double error = General.deadband(angle - currentAngle, HOOD_THRESHOLD_DEGREES);
            // Calculate speed
            double speed = 0;
            if (error != 0)
                speed = error < 0 ? 1 : -1;
            // Send speed to servo
            hood.set((speed + 1) / 2);
            // Return result
            return speed == 0;
        }
        return false;
    }

    public boolean setShooterVelocity(double targetVelocity) {
        if (targetVelocity != 0) {
            // Velocity is M/S
            double conversion = ((SHOOTER_ENCODER_TICKS * TALON_RATE) / (2 * Math.PI * SHOOTER_WHEEL_RADIUS));
            double input = targetVelocity * conversion;
            // Set is Tick/100ms
            shooter1.set(ControlMode.Velocity, input);
            // Calculate the current velocity
            double currentVelocity = shooter1.getSelectedSensorVelocity() / conversion;
            set("flywheel", String.valueOf(currentVelocity));
            // Check threshold
            return Math.abs(targetVelocity - currentVelocity) < SHOOTER_VELOCITY_THRESHOLD;
        } else {
            shooter1.set(ControlMode.PercentOutput, 0);
            return true;
        }
    }

    public boolean setTurretPosition(double angle) {
        // Calculate PID
        if (angle != 0) {
            double target = turretOffsetTicks + (int) (TURRET_ENCODER_TICKS * (angle / 360));
            double velocity = (target - turret.getSelectedSensorVelocity()) / (TALON_RATE);
            // Set the velocity
            turret.set(ControlMode.Velocity, velocity);
            // Check threshold
            return General.deadband(velocity, TURRET_THRESHOLD_VELOCITY) == 0;
        }
        return true;
    }

    public void setTurretVelocity(double speed) {
        turret.set(speed);
    }

    public double getHoodPosition() {
        double position = potentiometer.get();
        set("hood", String.valueOf(position));
        return position;
    }

    public int getShooterPosition() {
        int position = shooter1.getSelectedSensorPosition();
        set("shooter", String.valueOf(position));
        return position;
    }

    public double getTurretPosition() {
        double position = (turret.getSelectedSensorPosition() - turretOffsetTicks) / (TURRET_ENCODER_TICKS / 360.0);
        set("turret", String.valueOf(position));
        return position;
    }
}
