package frc.robot.efrat.systems;

import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.bobot.Subsystem;

public class Roller extends Subsystem {

    //    private WPI_TalonSRX motor;
//
//    public Roller() {
//        motor = new WPI_TalonSRX(8);
//    }
    private VictorSP motor;

    public Roller() {
        motor = new VictorSP(4);
    }

    public void set(double speed) {
        motor.set(speed);
    }
}
