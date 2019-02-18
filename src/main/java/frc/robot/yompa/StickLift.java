package frc.robot.yompa;

import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.bobot.Subsystem;

public class StickLift extends Subsystem {
    private VictorSP motor;

    public StickLift() {
        motor = new VictorSP(4);
    }

    public void setSpeed(double speed) {
        motor.set(speed/5);
    }
}
