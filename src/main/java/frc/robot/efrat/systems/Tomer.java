package frc.robot.efrat.systems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.bobot.Subsystem;

public class Tomer extends Subsystem {


    private static Tomer latest;
    private DoubleSolenoid claw, pivot;
    private DigitalInput cargoIndicator;

    public Tomer() {
        latest = this;
        claw = new DoubleSolenoid(2, 3);
        pivot = new DoubleSolenoid(4, 5);
//        cargoIndicator = new DigitalInput(PinMan.getNavDIO(5));
    }

    public static Tomer getInstance() {
        return latest;
    }

    public static void init() {
        if (getInstance() == null) new Tomer();
    }

    public boolean isCargoLoaded() {
        return true;
        // TODO uncomment that
//        return cargoIndicator != null && cargoIndicator.get();
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

    public void drop() {
        if (pivot != null) pivot.set(DoubleSolenoid.Value.kReverse);
    }

    public void lift() {
        if (pivot != null) pivot.set(DoubleSolenoid.Value.kForward);
    }
}
