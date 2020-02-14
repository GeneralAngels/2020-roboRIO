package frc.robot.kobi.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class KobiFeeder extends frc.robot.base.Module {

    private WPI_TalonSRX collector1, collector2;
    private WPI_TalonSRX feeder;

    public KobiFeeder(String id) {
        super(id);

        collector1 = new WPI_TalonSRX(16);
        collector2 = new WPI_TalonSRX(17);
        feeder = new WPI_TalonSRX(18);


    }

    public void setCollectorSpeed(double speed){
        collector1.set(speed);
        collector2.set(speed);
    }

    public void setFeederSpeed(double speed){
        feeder.set(speed);
    }
}
