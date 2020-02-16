package frc.robot.kobi.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

public class KobiFeeder extends frc.robot.base.Module {

    // Slider
    private WPI_TalonSRX slider;
    private DigitalInput maximumSwitch1, maximumSwitch2, minimumSwitch1, minimumSwitch2;

    // Feeder
    private WPI_TalonSRX feeder;

    // Collector (Roller-Gripper)
    private WPI_TalonSRX collector1, collector2;


    public KobiFeeder() {
        super("feeder");

        // Slider
        slider = new WPI_TalonSRX(15);
        minimumSwitch1 = new DigitalInput(9);
        minimumSwitch2 = new DigitalInput(8);
        maximumSwitch1 = new DigitalInput(7);
        maximumSwitch2 = new DigitalInput(6);

        // Collector
        collector1 = new WPI_TalonSRX(16);
        collector2 = new WPI_TalonSRX(17);

        // Feeder
        feeder = new WPI_TalonSRX(18);

        command("collector", new Command() {
            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                if (s.equals("in")) {
                    rollIn();
                    return new Tuple<>(true, "Collecting");
                } else if (s.equals("out")) {
                    rollOut();
                    return new Tuple<>(true, "Collecting");
                }
                return new Tuple<>(false, "Wrong parameter");
            }
        });

        command("feeder", new Command() {
            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                if (s.equals("in")) {
                    feedIn();
                    return new Tuple<>(true, "Feeding");
                } else if (s.equals("out")) {
                    feedOut();
                    return new Tuple<>(true, "Feeding");
                }
                return new Tuple<>(false, "Wrong parameter");
            }
        });

        command("slide", new Command() {
            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                if (s.equals("in")) {
                    return new Tuple<>(slideIn(), "Sliding");
                } else if (s.equals("out")) {
                    return new Tuple<>(slideOut(), "Sliding");
                }
                return new Tuple<>(false, "Wrong parameter");
            }
        });
    }

    public void rollIn() {
        collector1.set(0.2);
        collector2.set(0.2);
    }

    public void rollOut() {
        collector1.set(-0.2);
        collector2.set(-0.2);
    }

    public void feedIn() {
        feeder.set(0.2);
    }

    public void feedOut() {
        feeder.set(-0.2);
    }

    public boolean slideIn() {
        if (minimumSwitch1.get() || minimumSwitch2.get()) {
            slider.set(0);
            return true;
        } else {
            slider.set(-0.2);
            return false;
        }
    }

    private boolean slideOut() {
        if (maximumSwitch1.get() || maximumSwitch1.get()) {
            slider.set(0);
            return true;
        } else {
            slider.set(0.2);
            return false;
        }
    }
}
