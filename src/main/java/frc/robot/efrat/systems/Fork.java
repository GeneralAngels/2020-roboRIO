package frc.robot.efrat.systems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.bobot.Subsystem;
import frc.robot.bobot.utils.PinManager;

public class Fork extends Subsystem {


    private static Fork latest;
    private DoubleSolenoid solenoid;
    private DigitalInput cargoIndicator;

    public Fork() {
        latest = this;
        PinManager pinManager = new PinManager();
//        solenoid = new DoubleSolenoid(2, 3);
//        cargoIndicator = new DigitalInput(7);
    }

    public static Fork getInstance() {
        return latest;
    }

    public boolean isHatchLoaded() {
        // TODO acctual funt
        return false;
    }

    public boolean isCargoLoaded() {
        // TODO acctual funt
        return true;
    }

    public void open() {
        if (solenoid != null) solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void close() {
        if (solenoid != null) solenoid.set(DoubleSolenoid.Value.kReverse);
    }
}
