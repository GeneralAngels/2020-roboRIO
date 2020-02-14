package frc.robot.kobi.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class KobiFeeder extends frc.robot.base.Module {

    private WPI_TalonSRX collector1, collector2;
    private WPI_TalonSRX feeder;

    public KobiFeeder() {
        super("feeder");

        collector1 = new WPI_TalonSRX(16);
        collector2 = new WPI_TalonSRX(17);
        feeder = new WPI_TalonSRX(18);

        command("collector", new Command() {
            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                // Parse state
                boolean state = s.equals("on");
                // Set speed
                if (state){
                    setCollectorSpeed(0.5);
                }else{
                    setCollectorSpeed(0);
                }
                return new Tuple<>(true, "Speed set");
            }
        });

        command("feeder", new Command() {
            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                // Parse state
                boolean state = s.equals("on");
                // Set speed
                if (state){
                    setFeederSpeed(0.5);
                }else{
                    setFeederSpeed(0);
                }
                return new Tuple<>(true, "Speed set");
            }
        });
    }

    public void setCollectorSpeed(double speed){
        collector1.set(speed);
        collector2.set(speed);
    }

    public void setFeederSpeed(double speed){
        feeder.set(speed);
    }
}
