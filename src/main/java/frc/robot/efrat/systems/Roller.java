package frc.robot.efrat.systems;

import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.bobot.Subsystem;

public class Roller extends Subsystem {
    private static Roller latest;
    //    private WPI_TalonSRX motor;
//
//    public Roller() {
//        motor = new WPI_TalonSRX(8);
//    }
    private VictorSP motor;

    public Roller() {
        latest = this;
        motor = new VictorSP(4);
    }

    public static Roller getInstance() {
        return latest;
    }

    public void set(double speed) {
        motor.set(speed);
    }
}
