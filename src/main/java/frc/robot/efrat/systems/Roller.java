package frc.robot.efrat.systems;

import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.bobot.Subsystem;

public class Roller extends Subsystem {
    private static Roller latest;

    private VictorSP motor;

    public Roller() {
        latest = this;
        motor = new VictorSP(4);
    }

    public static void init() {
        if (getInstance() == null) new Roller();
    }

    public static Roller getInstance() {
        return latest;
    }

    public void set(double speed) {
        motor.set(speed);
    }

    public void open() {
        // TODO open bitch
    }

    public void close() {
        // TODO close bitch
    }
}
