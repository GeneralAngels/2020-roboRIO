package frc.robot.kobi.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class KobiFeeder extends frc.robot.base.Module {
    private WPI_TalonSRX collector, feeder;

    public KobiFeeder(String id) {
        super(id);
    }
}
