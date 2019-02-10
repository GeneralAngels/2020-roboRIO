package frc.robot.efrat.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.*;
import frc.robot.bobot.Subsystem;
import frc.robot.bobot.utils.PinManager;

public class Shiri extends Subsystem {

    private DoubleSolenoid solenoid;
    private WPI_TalonSRX slideMotor;
    public DigitalInput backReset, grab1, grab2;
    private Encoder slideEncoder;

    public Shiri() {
        PinManager pinManager = new PinManager();
        solenoid = new DoubleSolenoid(0, 1);
        slideMotor = new WPI_TalonSRX(4);
        slideEncoder = new Encoder(2, 3);
        grab1 = new DigitalInput(4);
        grab2 = new DigitalInput(5);
        backReset = new DigitalInput(6);
    }

    public boolean isHatchLoaded() {
        return grab1.get();
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
