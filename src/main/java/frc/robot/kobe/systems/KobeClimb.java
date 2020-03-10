package frc.robot.kobe.systems;

import com.ga2230.shleam.advanced.frc.FRCModule;
import edu.wpi.first.wpilibj.VictorSP;

public class KobeClimb extends FRCModule {

    private VictorSP climb0, climb1;

    public KobeClimb() {
        super("climb");

        climb0 = new VictorSP(0);
        climb1 = new VictorSP(1);
    }

    private void set(double speed) {
        climb0.set(speed);
        climb1.set(speed);
    }

    public void climb(Direction direction) {
        if (direction == Direction.Stop) {
            set(0);
        } else {
            if (direction == Direction.Up) {
                set(0.5);
            } else {
                set(-0.5);
            }
        }
    }

    public enum Direction {
        Up,
        Down,
        Stop
    }
}
