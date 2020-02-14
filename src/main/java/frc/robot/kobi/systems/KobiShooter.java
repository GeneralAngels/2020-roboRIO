package frc.robot.kobi.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.base.control.PID;

public class KobiShooter extends frc.robot.base.Module {

    private static final double TIMEOUT = 50;

    private static final double TURRET_TICKS_TOLERANCE = 50; // 50 tick tolerance

    private static final double GEAR = 1.0 / 3.0;
    private static final double ENCODER_TICKS = 4096;
    private static final double TALON_VELOCITY_RATE = 1000.0 / 100.0; // 10Hz

    private WPI_TalonSRX shooter1;
    private WPI_TalonSRX shooter2;
    private WPI_TalonSRX shooter3;

    private WPI_TalonSRX turret;
    private Servo hood;

    private PID motorsControlVelocity;

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

        shooter1.setSensorPhase(true);
        // turret.setSensorPhase(true); todo

        shooter1.configNominalOutputForward(0, 30);
        shooter1.configNominalOutputReverse(0, 30);
        shooter1.configPeakOutputForward(1, 30);
        shooter1.configPeakOutputReverse(-1, 30);
        shooter1.config_kP(0, 0, 30);
        shooter1.config_kI(0, 0, 30);
        shooter1.config_kD(0, 0, 30);
        shooter1.config_kF(0, 0.7, 30);

        shooter3.setInverted(true);

        shooter2.follow(shooter1);
        shooter3.follow(shooter1);
//        motorsControlVelocity = new PID("motorControlVelocity", 0.005, 0.007, 0.05, 0.05); // todo not finished. pay attention! - very small k's!

        command("turret", new Command() {

            private boolean finished = true;
            private int targetPosition = 0;

            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                double delta = Double.parseDouble(s);
                // Set position
                if (finished) {
                    targetPosition = setTurretPosition(delta);
                    finished = false;
                } else {
                    if (Math.abs(getTurretPosition() - targetPosition) < TURRET_TICKS_TOLERANCE) {
                        finished = true;
                        return new Tuple<>(true, "Finished");
                    }
                }
                return new Tuple<>(false, "Moving turret");
            }
        });
    }

    public void setVelocity(double velocity) {
        // Velocity is RPM
        double input = velocity * GEAR * (ENCODER_TICKS / (60 * TALON_VELOCITY_RATE));
        // Input is (?)
        shooter1.set(ControlMode.Velocity, input);

        set("shooter-encoder", String.valueOf(getPosition()));
    }

    public int setTurretPosition(double position) {
        // Position is degrees
        int input = (int) ((position / 360.0) * GEAR * ENCODER_TICKS);
        turret.set(ControlMode.Position, input);

        set("turret-encoder", String.valueOf(getTurretPosition()));
        return input;
    }

    public int getTurretPosition() {
        return turret.getSelectedSensorPosition();
    }

    public void setHoodPosition(double position) {
        hood.set(position);
    }

    public int getPosition() {
        return shooter1.getSelectedSensorPosition();
    }
}
