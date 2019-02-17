package frc.robot.efrat.systems;

import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.bobot.Subsystem;

public class Roller extends Subsystem {
    private static Roller latest;
    //    public double x =0;
//    public double y = 0;
    //    private WPI_TalonSRX motor;
//
//    public Roller() {
//        latest = this;
//        motor = new WPI_TalonSRX(8);
//    }
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
