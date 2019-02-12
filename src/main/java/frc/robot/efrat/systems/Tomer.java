package frc.robot.efrat.systems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.bobot.Subsystem;
import frc.robot.bobot.utils.PinManager;

public class Tomer extends Subsystem {


    private static Tomer latest;
    private DoubleSolenoid claw, pivot;
    private DigitalInput cargoIndicator;

    public Tomer() {
        latest = this;
        PinManager pinManager = new PinManager();
//        claw = new DoubleSolenoid(2, 3);
//        cargoIndicator = new DigitalInput(7);
    }

    public static Tomer getInstance() {
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

    public boolean isDown() {
        return pivot != null && pivot.get() == DoubleSolenoid.Value.kReverse;
    }

    public boolean isOpened() {
        return claw != null && claw.get() == DoubleSolenoid.Value.kForward;
    }

    public void open() {
        if (claw != null) claw.set(DoubleSolenoid.Value.kForward);
    }

    public void close() {
        if (claw != null) claw.set(DoubleSolenoid.Value.kReverse);
    }

    public void drop(){
        if(pivot!=null)pivot.set(DoubleSolenoid.Value.kReverse);
    }

    public void lift(){
        if(pivot!=null)pivot.set(DoubleSolenoid.Value.kForward);
    }
}
