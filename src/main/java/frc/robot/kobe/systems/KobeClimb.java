package frc.robot.kobe.systems;

import com.ga2230.shleam.advanced.frc.FRCModule;
import edu.wpi.first.wpilibj.VictorSP;

public class KobeClimb extends FRCModule {

    private VictorSP climb0, climb1;

    public KobeClimb() {
        super("climb");
    }
}
