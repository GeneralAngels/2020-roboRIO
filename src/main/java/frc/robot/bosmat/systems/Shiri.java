package frc.robot.bosmat.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.base.Module;
import frc.robot.base.control.PID;

/**
 * What's this?
 * Claw class for Bosmat (2230 2019 Robot)
 */

public class Shiri extends Module {

    private static final double DISTANCE = 0.66;
    private static final double RADIUS = 0.0191; //TODO: check value
    private static final double TICKS_PER_REVOLUTIONS = 1024; //TODO: check value
    private static final double ENC_TO_METERS = (2 * 3.14 * RADIUS) / (4 * TICKS_PER_REVOLUTIONS);
    private static Shiri latest;
    public double encoder = 0;
    private DoubleSolenoid hatch;
    private double targetX = -1;
    private double currentX = 0;

    public Shiri() {
        latest = this;
        hatch = new DoubleSolenoid(0, 4, 7);
    }

    public static void init() {
        if (getInstance() == null) new Shiri();
    }

    public static Shiri getInstance() {
        return latest;
    }

    public void open() {
        if (hatch != null) hatch.set(DoubleSolenoid.Value.kForward);
    }

    public void close() {
        if (hatch != null) hatch.set(DoubleSolenoid.Value.kReverse);
    }

    public int sign(double a) {
        if (a > 0)
            return 1;
        else if (a < 0) return -1;
        return 0;
    }

    public boolean in_place(double x, double diffrenceX) {
        if (targetX > 0.36) {
            return (currentX - (x - 0.23) > diffrenceX);
        } else {
            return ((x - 0.23) - currentX) > diffrenceX;
        }
    }
}
