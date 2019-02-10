package frc.robot.efrat.systems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.bobot.Subsystem;
import frc.robot.bobot.utils.PinManager;

public class Fork extends Subsystem {

    private DoubleSolenoid solenoid;
    private DigitalInput cargoIndicator;

    public Fork() {
        PinManager pinManager = new PinManager();
        solenoid = new DoubleSolenoid(2, 3);
//        cargoIndicator = new DigitalInput(7);
    }

    public boolean isHatchLoaded() {
        // TODO acctual funt
        return false;
    }

    public boolean isCargoLoaded() {
        // TODO acctual funt
        return false;
    }

    public void open() {
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void close() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }
}
