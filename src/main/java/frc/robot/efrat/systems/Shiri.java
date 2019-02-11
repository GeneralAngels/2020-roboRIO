package frc.robot.efrat.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.bobot.Subsystem;
import frc.robot.bobot.utils.PinManager;

public class Shiri extends Subsystem {

    private static Shiri latest;
    public DigitalInput backReset, grab1, grab2;
    private DoubleSolenoid solenoid;
    private WPI_TalonSRX slideMotor;
    private Encoder slideEncoder;

    public Shiri() {
        latest = this;
        PinManager pinManager = new PinManager();
        solenoid = new DoubleSolenoid(0, 1);
        slideMotor = new WPI_TalonSRX(4);
        slideEncoder = new Encoder(2, 3);
//        grab1 = new DigitalInput(0);
//        grab2 = new DigitalInput(1);
//        backReset = new DigitalInput(6);
    }

    public static Shiri getInstance() {
        return latest;
    }

    public boolean isHatchLoaded() {
        if (grab1 == null || grab2 == null) return false;
        return grab1.get() && grab2.get();
    }

    public void open() {
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void close() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void set(double speed) {
        slideMotor.set(speed);
    }
}
