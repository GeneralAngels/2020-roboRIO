package frc.robot.kobi.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import frc.robot.base.control.PID;

public class KobiShooter extends frc.robot.base.Module {

    private static final double TIMEOUT = 50;

    private static final double TICK_ROUNDS = (2 * Math.PI / 4096);

    private WPI_TalonSRX motor1;
    private WPI_TalonSRX motor2;
    private WPI_TalonSRX motor3;
    private PID motorsControlVelocity;
    private PowerDistributionPanel pdp;
    private int encoder = 0;
    private int previousEncoder = 0;

    private double previousTime = 0;

    public KobiShooter() {
        super("shooter");

        motor1 = new WPI_TalonSRX(20);
        motor2 = new WPI_TalonSRX(21);
        motor3 = new WPI_TalonSRX(22);

        motor1.setSelectedSensorPosition(1);

        motor1.config_kP(0, 0);
        motor1.config_kI(0, 0);
        motor1.config_kD(0, 0);
        motor1.config_kF(0, 0.3);

//        motor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        motorsControlVelocity = new PID("motorControlVelocity", 0.005, 0.007, 0.05, 0.05); // todo not finished. pay attention! - very small k's!
        pdp = new PowerDistributionPanel(1);


        command("shoot", new Command() {
            @Override
            public String execute(String s) throws Exception {
                return "NI";
            }
        });
        command("move", new Command() {
            @Override
            public String execute(String s) throws Exception {
                return "NI";
            }
        });
    }

    public void applyPower(double power) {
        double motorOutput = power / pdp.getVoltage();
        if (motorOutput < -0.1)
            motorOutput = -0.1;
        motor1.set(motorOutput);
        motor2.set(motorOutput);
        motor3.set(-motorOutput);
    }

    public void setVelocity(double velocity) {
//        motorsControlVelocity.updateDelta();
//        applyPower(motorsControlVelocity.PIDVelocity(TICK_ROUNDS * getPosition(), velocity / 0.0722));
        motor1.set(ControlMode.Velocity, velocity);
        motor2.set(motor1.get());
        motor3.set(-motor1.get());
        set("speed", String.valueOf(motorsControlVelocity.getDerivative()));
    }

    public int getPosition() {
        set("encoder", String.valueOf(this.encoder));
        set("delcoder", String.valueOf(this.encoder - this.previousEncoder));
        previousEncoder = encoder;
        previousTime = millis();
        if ((encoder = motor1.getSelectedSensorPosition()) == 0)
            encoder = previousEncoder;
        return encoder;
    }
}
